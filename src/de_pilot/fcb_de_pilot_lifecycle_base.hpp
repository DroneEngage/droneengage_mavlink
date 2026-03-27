#ifndef FCB_DE_PILOT_LIFECYCLE_BASE_H_
#define FCB_DE_PILOT_LIFECYCLE_BASE_H_

#include <cstdint>
#include "../fcb_main.hpp"

namespace de {
namespace fcb {
namespace depilot {

class CDEPilotLifecycleBase {
public:
    virtual ~CDEPilotLifecycleBase() = default;

    // Pure virtual lifecycle interface
    virtual void init() = 0;
    virtual void update() = 0;
    virtual void uninit() = 0;
    virtual void readConfigParameters() = 0;
    virtual void reloadParametersIfConfigChanged() = 0;

    // Generic phase interface
    virtual void setPhase(int phase) = 0;
    virtual int getPhase() const = 0;

    // Active state management
    virtual void setActive(bool active) = 0;
    virtual bool getActive() const = 0;

    // Identifier for logging and events
    virtual std::string getName() const = 0;

protected:
    // Common member variables that all lifecycle participants can use
    bool m_active = false;
    int m_generic_phase = 0;
    uint64_t m_phase_start_time = 0;
    uint64_t m_last_update_time = 0;

    de::fcb::CFCBMain &m_fcbMain = de::fcb::CFCBMain::getInstance();
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif // FCB_DE_PILOT_LIFECYCLE_BASE_H_
