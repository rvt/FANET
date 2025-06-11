#pragma once

#include "fanet.hpp"
#include "etl/optional.h"
#include "etl/vector.h"
#include "etl/bit_stream.h"
#include "etl/variant.h"
#include "etl/random.h"

#include "header.hpp"
#include "address.hpp"
#include "txFrame.hpp"
#include "tracking.hpp"
#include "name.hpp"
#include "message.hpp"
#include "utils.hpp"

#include "groundTracking.hpp"
#include "blockAllocator.hpp"
#include "packetParser.hpp"
#include "neighbourTable.hpp"
#include "connector.hpp"

namespace FANET
{
    /**
     * @brief Protocol class for handling FANET communication.
     *
     * This class manages the sending and receiving of FANET packets, including handling
     * acknowledgments, forwarding, and maintaining a neighbor table.
     */
    class Protocol
    {
    public:
        struct Stats 
        {
            uint32_t rx = 0;                 // All packets received
            uint32_t txSuccess = 0;          // All packets transmitted
            uint32_t txFailed = 0;           // An attempted transmission failed
            uint32_t processed = 0;          // Packets passed to the application stack to be processed
            uint32_t forwarded = 0;          // All packets that were forwarded
            uint32_t fwdMinRssiDrp = 0;      // Packets discarded due to Rssi being too good
            uint32_t fwdNeighborDrp = 0;     // Packets discarded due to no neighbor in neighbor table
            uint32_t fwdDbBoostDrop = 0;     // Pkts dropped from txQueue with subsequent good rssi
            uint32_t fwdDbBoostWeak = 0;     // Pkts forwarded that was in our TX queue due seeing a rxmit with a poor rssi
            uint32_t fwdDropAirtime = 0;     // Packets dropped due to too much time on the air recently.
            uint32_t rxFromUsDrp = 0;        // Dropped packets from our own Mac
            uint32_t txAck = 0;              // Number of Acks sent
            uint32_t neighborTableSize = 0;  // Number of neighbors currently in our neighbor table
        };
    protected:

        static constexpr int32_t MAC_SLOT_MS = 20;

        static constexpr int32_t MAC_TX_MINPREAMBLEHEADERTIME_MS = 15;
        static constexpr int32_t MAC_TX_TIMEPERBYTE_MS = 2;
        static constexpr int32_t MAC_TX_ACKTIMEOUT = 1000;
        static constexpr int32_t MAC_TX_RETRANSMISSION_TIME = 1000;
        static constexpr uint8_t MAC_TX_RETRANSMISSION_RETRYS = 3;
        static constexpr int32_t MAC_TX_BACKOFF_EXP_MIN = 7;
        static constexpr int32_t MAC_TX_BACKOFF_EXP_MAX = 12;

        static constexpr int16_t MAC_FORWARD_MAX_RSSI_DBM = -90; // todo test
        static constexpr int32_t MAC_FORWARD_MIN_DB_BOOST = 20;
        static constexpr int32_t MAC_FORWARD_DELAY_MIN = 100;
        static constexpr int32_t MAC_FORWARD_DELAY_MAX = 300;
        static constexpr int32_t FANET_MAX_NEIGHBORS = 30;

        static constexpr int32_t APP_TYPE1OR7_MINTAU_MS = 250;
        static constexpr int32_t APP_TYPE1OR7_TAU_MS = 5000;

        static constexpr int32_t FANET_CSMA_MIN = 20;
        static constexpr int32_t FANET_CSMA_MAX = 40;

        static constexpr int32_t MAC_MAXNEIGHBORS_4_TRACKING_2HOP = 5;
        static constexpr int32_t MAC_CODING48_THRESHOLD = 8;

        static constexpr int16_t MAC_DEFAULT_TX_BACKOFF = 1000;

        // Random number generator for random times
        etl::random_xorshift random; // XOR-Shift PRNG from ETL

        // Pool for all frames that need to be sent
        using TxPool = BlockAllocator<FANET::TxFrame<uint8_t>, 50, 16>;
        TxPool txPool;

        // Table with received neighbors
        NeighbourTable<FANET_MAX_NEIGHBORS> neighborTable_;

        // User's own address
        Address ownAddress_{1}; // Default to 1 to ensure 'ownAddress_' is not broadcast
        // When set to true, the protocol handler will forward received packages when applicable
        bool doForward = true;

        // When has been set, then this is taken into consideration when to send the next packet
        // This is like CSMA in the old protocol.
        uint32_t cmcaNextTx = 0;
        uint8_t carrierBackoffExp = MAC_TX_BACKOFF_EXP_MIN;
        Airtime airtime;

        // Connector for the application, e.g., the interface between the FANET protocol and the application
        Connector *connector;

        // Statistics for the protocol's operation
        Stats stats_;

        /**
         * @brief Build an acknowledgment packet from an existing frame.
         * @param frame The frame to acknowledge.
         * @return The acknowledgment frame.
         */
        auto buildAck(TxFrame<const uint8_t> &frame)
        {
            // fmac.260
            Packet<1> ack;
            ack.source(ownAddress_).destination(frame.source());

            // fmac.265
            /* only do a 2 hop ACK in case it was requested and we received it via a two hop link (= forward bit is not set anymore) */
            if (frame.ackType() == ExtendedHeader::AckType::TWOHOP && !frame.forward())
            {
                ack.forward(true);
            }

            return ack.buildAck();
        }

        /**
         * @brief decide if the time is reached
         */
        bool timeReached(uint32_t tick, uint32_t time)
        {
            return static_cast<int32_t>(tick - time) >= 0;
        }

        /**
         * @brief Find a frame in the TX pool that matches the given buffer.
         * Matches are based on source, data.size, destination, type and the payload
         *
         * @param buffer The buffer to match.
         * @return A pointer to the matching TX frame, or nullptr when no match is found.
         */
        TxFrame<uint8_t> *frameInTxPool(const TxFrame<const uint8_t> &other)
        {
            // clang-format off
            auto it = etl::find_if(txPool.begin(), txPool.end(),
                [&other](auto block)
                {
                    // frame.162
                    if (block.source() != other.source()) {return false; }
                    if (block.data().size() != other.data().size())  {return false; }
                    if (block.destination() != other.destination())  {return false; }
                    if (block.type() != other.type())  {return false; }
                    if (!etl::equal(block.payload(), other.payload()))  {return false; }
                    return true;
                });
            // clang-format on

            if (it != txPool.end())
            {
                return &(*it);
            }

            return nullptr;
        }

        /**
         * @brief Remove any pending frame that waits on an ACK from a host
         *
         * @param destination The destination address of the acknowledged frames
         * @return The id if the package in the pool, 0 if nothing found or the packet did not have an id
         */
        uint16_t removeDeleteAckedFrame(const Address &source)
        {
            uint16_t id = 0;
            for (auto it = txPool.begin(); it != txPool.end();)
            {
                if (it->destination() == source && it->ackType() != ExtendedHeader::AckType::NONE)
                {
                    id = it->id();
                    it = txPool.remove(it);
                }
                else
                {
                    ++it;
                }
            }
            return id;
        }

        /**
         * @brief Get the next TX block that can be transmitted.
         *
         * This function finds the next TX block that can be transmitted in the following order:
         * a) Find any self
         * b) Find any priority packed (tracking packets)
         * c) Find any acknowledgment packet.
         * d) Find any other packet.
         *
         * Packages of equal priority will be ordered by nextTx time, lowest first
         *
         * @param timeMs The current time in milliseconds.
         * @return A pointer to the next TX frame, or nullptr if no frame is available.
         */
        TxFrame<uint8_t> *getNextTxFrame(uint32_t timeMs)
        {
            TxFrame<uint8_t> *nextFrame = nullptr;
            uint8_t highestPriority = 4; // 1 = self, 2 = priority, 3 = ack, 4 = other
            uint32_t earliestTime = UINT32_MAX;

            for (auto it = txPool.begin(); it != txPool.end(); ++it)
            {
                if (!timeReached(timeMs, it->nextTx()))
                    continue;

                int priorityLevel = 4;
                if (it->self())
                {
                    priorityLevel = 1;
                }
                else if (it->isTrackingType())
                {
                    priorityLevel = 2;
                }
                else if (it->type() == Header::MessageType::ACK)
                {
                    priorityLevel = 3;
                }

                // Select frame if:
                // 1. It has a higher priority, OR
                // 2. It has the same priority but an earlier nextTx() time
                if (priorityLevel < highestPriority ||
                    (priorityLevel == highestPriority && it->nextTx() < earliestTime))
                {
                    nextFrame = &(*it);
                    highestPriority = priorityLevel;
                    earliestTime = it->nextTx();
                }
            }

            return nextFrame;
        }

        auto sendFrame(TxFrame<uint8_t> *frm)
        {
            struct ret
            {
                bool isSend;
                uint16_t lengthBytes;
            };
            auto cr = neighborTable_.size() < MAC_CODING48_THRESHOLD ? 8 : 5;
            uint16_t lengthBytes = frm->data().size();
            auto airTime = FANET::LoraAirtime(lengthBytes, 7, 250, cr - 4);
            airtime.set(connector->fanet_getTick(), airTime);
            // printf("Length bytes : length:%d time:%d airTime:%d\n", lengthBytes, airTime, airtime.get(connector->fanet_getTick()));
            return ret{
                connector->fanet_sendFrame(cr, frm->data()),
                lengthBytes};
        }

        void ackReceived(uint16_t id)
        {
            return connector->fanet_ackReceived(id);
        }

    public:
        /**
         * @brief Constructor for the Protocol class.
         * @param connector_ The connector interface for the application.
         */
        Protocol(Connector *connector_) : connector(connector_)
        {
            init();
        }

        void init()
        {
            random.initialise(connector->fanet_getTick());
            neighborTable_.clear();
            txPool.clear();
        }

        void ownAddress(const Address &adress)
        {
            // 0x and 0xFFFFFF are reserved
            if (adress == Address{0x00, 0x0000} || adress == Address{0xFF, 0xFFFF})
            {
                return;
            }
            ownAddress_ = adress;
        }

        const Address &ownAddress()
        {
            return ownAddress_;
        }

        const TxPool &pool() const
        {
            return txPool;
        }

        const NeighbourTable<FANET_MAX_NEIGHBORS> &neighborTable() const
        {
            return neighborTable_;
        }

        /**
         * @brief Get the average airtime
         */
        uint32_t airTime() const
        {
            return airtime.getAverage();
        }

        /**
         * @brief Send a FANET packet.
         * @tparam MAXFRAMESIZE The size of the message payload.
         * @tparam MAXFRAMESIZE The size of the name payload.
         * @param packet The packet to send.
         * @param id ID of this packet, can be used if you request an ack and to know if the packet was received
         */
        template <size_t MAXFRAMESIZE>
        void sendPacket(Packet<MAXFRAMESIZE> &packet, uint16_t id = 0, bool strict = true)
        {
            uint8_t numTx;
            if (strict)
            {
                // Ensure source is always our own address
                packet.source(ownAddress_);
                auto eh = packet.extendedHeader().value_or(ExtendedHeader{});
                // Forward must be true when extended header has a ackType none NONE
                if (eh.ack() != ExtendedHeader::AckType::NONE)
                {
                    packet.forward(true);
                    numTx = MAC_TX_RETRANSMISSION_RETRYS;
                }
                else
                {
                    numTx = 0;
                }
            }

            auto v = packet.build();
            auto txFrame = TxFrame<uint8_t>{{v.data(), v.size()}}.self(true).id(id).nextTx(connector->fanet_getTick()).numTx(numTx);
            txPool.add(txFrame);
        }

        /**
         * @brief Handle a received FANET packet.
         * @tparam MAXFRAMESIZE The size of the message payload.
         * @tparam MAXFRAMESIZE The size of the name payload.
         * @param rssddBm The received signal strength in dBm.
         * @param buffer The byte buffer containing the packet data.
         * @return The MessageType
         */
        Header::MessageType handleRx(int16_t rssddBm, etl::span<const uint8_t> buffer)
        {
            stats_.rx++; // All packets received

            // START: OK
            // static constexpr size_t MAC_PACKET_SIZE = ((MAXFRAMESIZE > MAXFRAMESIZE) ? MAXFRAMESIZE : MAXFRAMESIZE) + 12; // 12 Byte for maximum header size
            auto timeMs = connector->fanet_getTick();

            auto packet = TxFrame<const uint8_t>{buffer};
            // packet.print();

            auto destination = packet.destination();

            // fmac.283 Perhaps move this to some maintenance task?
            neighborTable_.removeOutdated(timeMs);

            // Drop packages forwarded to us
            if (packet.source() == ownAddress_)
            {
                stats_.rxFromUsDrp++;
                return packet.type();
            }
            stats_.processed++;

            // fmac.322
            // addOrUpdate will guarantee this one is added, and any old one is removed
            neighborTable_.addOrUpdate(packet.source(), timeMs);

            stats_.neighborTableSize = neighborTable_.size();

            // fmac.326
            // Decide if we have seen this frame already in the past, if so decide what to do with the frame in our buffer
            // This concerns forwarding of packets from other FANET devices
            auto frmList = frameInTxPool(packet);
            if (frmList)
            {
                /* frame already in tx queue */
                if (rssddBm > (frmList->rssi() + MAC_FORWARD_MIN_DB_BOOST))
                {
                    /* received frame is at least 20dB better than the original -> no need to rebroadcast */
                    stats_.fwdDbBoostDrop++;
                    txPool.remove(frmList);
                }
                else
                {
                    /* adjusting new departure time */
                    // fmac.346
                    stats_.fwdDbBoostWeak++;
                    frmList->nextTx(connector->fanet_getTick() + random.range(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX));
                }
            }
            // END: OK
            else
            {
                // When the package was destined to us or broadcast
                // Note don't generate an ack for a packet that is coming back to us that genertaed on our own
                // fmac.351
                if ((destination == Address{} || destination == ownAddress_) && packet.source() != ownAddress_)
                {
                    // fmac.353
                    // When we receive an ack in broadcast or to us, but the frame was not found in our pool
                    if (packet.type() == Header::MessageType::ACK)
                    {
                        // fmac.cpp 356
                        auto id = removeDeleteAckedFrame(packet.source());

                        // Inform the application only if the id is != 0
                        if (id)
                        {
                            ackReceived(id);
                        }
                    }
                    else
                    {
                        // fmac.361
                        // When ACK was requested
                        if (packet.ackType() != ExtendedHeader::AckType::NONE)
                        {
                            // fmac.362
                            auto v = buildAck(packet);
                            txPool.add(TxFrame<uint8_t>{v}.nextTx(timeMs));
                            stats_.txAck++;
                        }
                    }
                }

                // fmac.371
                if (doForward && packet.forward())
                {
                    if(rssddBm > MAC_FORWARD_MAX_RSSI_DBM) 
                    {
                        stats_.fwdMinRssiDrp++; // Packets not forwarding due to Rssi being too good
                    } else if(destination != Address{} && !neighborTable_.lastSeen(destination))
                    {
                        stats_.fwdNeighborDrp++; // Packets discarded due to no neighbor in neighbor table
                    } else if(airtime.get(timeMs) > 500) 
                    {
                        stats_.fwdDropAirtime++;
                    } else
                    {
                        /* generate new tx time */
                        auto nextTx = timeMs + random.range(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);
                        auto numTx = packet.ackType() != ExtendedHeader::AckType::NONE ? 1 : 0;

                        /* add to list */
                        // fmac.368
                        // fmac.368
                        auto txFrame = TxFrame<uint8_t>{etl::span<uint8_t>(const_cast<uint8_t *>(buffer.data()), buffer.size())}
                                        .rssi(rssddBm)
                                        .numTx(numTx)
                                        .nextTx(nextTx)
                                        .forward(false);
                        txPool.add(txFrame);
                        stats_.forwarded++;
                    }
                }
            }

            return packet.type();
        }

        /**
         * @brief Handle any packets in the queue.
         *
         * This can be called at regular intervals, or it can be called based on the time returned by this function when it
         * thinks a packet can be sent. This function might not always send a packet even if there are packets in the queue.
         *
         * @return The time in milliseconds until the next packet can be sent.
         */
        uint32_t handleTx()
        {
            auto timeMs = connector->fanet_getTick();

            // fmac.403
            if (!timeReached(timeMs, cmcaNextTx))
            {
                return cmcaNextTx;
            }

            // Get TxFrame if available
            // fmac.417
            auto frm = getNextTxFrame(timeMs);
            if (frm == nullptr)
            {
                return timeMs + MAC_DEFAULT_TX_BACKOFF;
            }

            // fmac.414
            // This condition is similar to app_tx in the original code where tracking packages are send
            // In this implementation everything is added to the txPool
            if (frm->self() && frm->isTrackingType())
            {
                // fmac.421
                // Note: I find it odd that we set forward based on neighborTable_ table
                bool setForward = neighborTable_.size() < MAC_MAXNEIGHBORS_4_TRACKING_2HOP;
                frm->forward(setForward);
                auto status = sendFrame(frm);
                
                // Update stats
                if(status.isSend) 
                {
                    stats_.txSuccess++;
                }
                else
                {
                    stats_.txFailed++;
                }

                txPool.remove(frm);
                carrierBackoffExp = MAC_TX_BACKOFF_EXP_MIN;
                cmcaNextTx = timeMs + MAC_TX_MINPREAMBLEHEADERTIME_MS + (status.lengthBytes * MAC_TX_TIMEPERBYTE_MS);
                return cmcaNextTx;
            }

            // Validate if there is time for any other frames
            // fmac.428
            auto airtimeMs = airtime.get(timeMs);
            // printf("Air time : %dms\n", airtimeMs);
            if (airtimeMs >= 900)
            {
                return timeMs + MAC_DEFAULT_TX_BACKOFF;
            }

            // fmac.446
            // Clean up the TX queue of frames that where never acked
            if (frm->ackType() != ExtendedHeader::AckType::NONE && frm->numTx() == 0)
            {
                txPool.remove(frm);
                // Recursive  to handle next frame if it's in the pool
                return handleTx();
            }

            // fmac.457
            /* unicast frame w/o forwarding and it is not a direct neighbor */
            auto destination = frm->destination();
            if (frm->forward() == false &&
                destination != Address{} &&
                neighborTable_.lastSeen(destination) == 0)
            {
                frm->forward(true);
            }

            /////////  Send data
            // fmac.502
            auto status = sendFrame(frm);
            timeMs = connector->fanet_getTick();

            // fmac 505
            if (status.isSend)
            {
                stats_.txSuccess++;
                // fmac.522
                // Remove from if no ACK was requested, and not ours
                if (frm->ackType() == ExtendedHeader::AckType::NONE || frm->source() != ownAddress_)
                {
                    // fmac.524
                    txPool.remove(frm);
                }
                else
                {
                    // fmac.529
                    // THis will only be true when packages are send by some third party interface in fanet_cmd_state??
                    frm->numTx(frm->numTx() - 1);
                    if (frm->numTx() > 0)
                    {
                        // fmac.531
                        frm->nextTx(timeMs + (MAC_TX_RETRANSMISSION_TIME * (MAC_TX_RETRANSMISSION_RETRYS - frm->numTx())));
                    }
                    else
                    {
                        // fmac.533
                        frm->nextTx(timeMs + MAC_TX_ACKTIMEOUT);
                    }
                }

                carrierBackoffExp = MAC_TX_BACKOFF_EXP_MIN;
                cmcaNextTx = timeMs + MAC_TX_MINPREAMBLEHEADERTIME_MS + (status.lengthBytes * MAC_TX_TIMEPERBYTE_MS);
                return cmcaNextTx;
            }
            else
            {
                stats_.txFailed++;
                /* channel busy, increment backoff exp */
                if (carrierBackoffExp < MAC_TX_BACKOFF_EXP_MAX)
                {
                    carrierBackoffExp++;
                }

                /* next tx try */
                cmcaNextTx = timeMs + random.range(1 << (MAC_TX_BACKOFF_EXP_MIN - 1), 1 << carrierBackoffExp);
                return cmcaNextTx;
            }

            return timeMs + MAC_DEFAULT_TX_BACKOFF;
        }

        void printTxPool() const
        {
            for (auto it = txPool.begin(); it != txPool.end(); it++)
            {
                it->print();
            }
        }

        const Stats& stats() const { return stats_; }

        size_t txPoolSize() const { return static_cast<int>(txPool.getAllocatedBlocks().size()); }
    };

};
