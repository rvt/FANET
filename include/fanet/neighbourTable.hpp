#pragma once

#include <stdint.h>
#include "etl/vector.h"

#include "address.hpp"

namespace FANET
{
    static constexpr uint32_t NEIGHBOR_MAX_TIMEOUT_MS = 4 * 60 * 1000 + 10000; // 4min + 10sek

    template <size_t FANET_MAX_NEIGHBORS>
    class NeighbourTable
    {
    private:
        struct Neighbour
        {
            Address address;
            uint32_t lastSeen;
        };
        etl::vector<Neighbour, FANET_MAX_NEIGHBORS> neighborTable_;

    public:
        void clear()
        {
            return neighborTable_.clear();
        }
        size_t size() const
        {
            return neighborTable_.size();
        }

        void addOrUpdate(Address address, uint32_t lastSeen)
        {
            // THis is to simply the code, and 1ms is not a concern
            // So that lastSeen will always return a value
            if (neighborTable_.full())
            {
                removeOldest();
            }

            auto it = std::find_if(neighborTable_.begin(), neighborTable_.end(), [&address](const Neighbour &n)
                                   { return n.address == address; });
            if (it != neighborTable_.end())
            {
                it->lastSeen = lastSeen;
            }
            else
            {
                neighborTable_.push_back(Neighbour{address, lastSeen});
            }
        }

        void remove(const Address &address)
        {
            neighborTable_.erase(std::remove_if(neighborTable_.begin(), neighborTable_.end(), [&address](const Neighbour &neighbour)
                                               { return neighbour.address == address; }),
                                neighborTable_.end());
        }

        uint32_t lastSeen(const Address &address) const
        {
            auto it = std::find_if(neighborTable_.begin(), neighborTable_.end(), [&address](const Neighbour &neighbour)
                                   { return neighbour.address == address; });
            if (it != neighborTable_.end())
            {
                return it->lastSeen;
            }
            return 0;
        }


        void removeOldest()
        {
            auto it = std::min_element(neighborTable_.begin(), neighborTable_.end(), [](const Neighbour &a, const Neighbour &b)
                                       { return a.lastSeen < b.lastSeen; });
            if (it != neighborTable_.end())
            {
                neighborTable_.erase(it);
            }
        }


        void removeOutdated(uint32_t timeMs)
        {
            neighborTable_.erase(std::remove_if(neighborTable_.begin(), neighborTable_.end(), [&timeMs](const Neighbour &neighbour)
                                               { 
                                                uint32_t diff = timeMs - neighbour.lastSeen;
                                                return diff >  NEIGHBOR_MAX_TIMEOUT_MS; }),
                                neighborTable_.end());
        }

        const etl::ivector<Neighbour> &neighborTable() const {
          return neighborTable_;
         }
    };

}