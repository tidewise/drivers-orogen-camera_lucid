#ifndef camera_lucid_SYSTEM_HPP
#define camera_lucid_SYSTEM_HPP

#include "Arena/ArenaApi.h"
#include <mutex>

namespace camera_lucid {
    /** Thread-safe access to Arena's system API
     */
    class System {
        static System* s_system;
        Arena::ISystem* m_system;

        std::mutex m_sync;

        System()
        {
            m_system = Arena::OpenSystem();
        }

        ~System()
        {
            Arena::CloseSystem(m_system);
        }

    public:
        static System& Instance()
        {
            return *s_system;
        }

        static void Init()
        {
            s_system = new System();
        }

        static void Deinit()
        {
            if (s_system) {
                delete s_system;
                s_system = nullptr;
            }
        }

        void DestroyDevice(Arena::IDevice* device)
        {
            std::lock_guard<std::mutex> lock(m_sync);
            m_system->DestroyDevice(device);
        }

        void UpdateDevices(int delay)
        {
            std::lock_guard<std::mutex> lock(m_sync);
            m_system->UpdateDevices(delay);
        }

        std::vector<Arena::DeviceInfo> GetDevices()
        {
            std::lock_guard<std::mutex> lock(m_sync);
            return m_system->GetDevices();
        }

        Arena::IDevice* CreateDevice(Arena::DeviceInfo const& info)
        {
            std::lock_guard<std::mutex> lock(m_sync);
            return m_system->CreateDevice(info);
        }
    };
}

#endif