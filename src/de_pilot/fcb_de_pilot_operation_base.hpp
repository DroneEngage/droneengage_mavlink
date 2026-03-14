#ifndef FCB_de_pilot_operation_BASE_H_
#define FCB_de_pilot_operation_BASE_H_

#include <cstdint>

namespace de {
namespace fcb {
namespace depilot {

class CDEPilotOperationBase {
public:
    virtual ~CDEPilotOperationBase() = default;

    // Pure virtual interface that all derived classes must implement
    virtual void init() = 0;
    virtual void update() = 0;
    virtual void uninit() = 0;
    virtual void readConfigParameters() = 0;

    // Generic phase interface - derived classes handle their specific phases internally
    virtual void setPhase(int phase) = 0;
    virtual int getPhase() const = 0;

    // Active state management
    virtual void setActive(bool active) = 0;
    virtual bool getActive() const = 0;

    virtual bool isCompleted() = 0;
protected:
    // Common member variables that all operations can use
    bool m_active = false;
    int m_generic_phase = 0;  // Generic phase representation
    uint64_t m_phase_start_time = 0;
    uint64_t m_last_update_time = 0;
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif // FCB_m_de_pilot_operation_BASE_H_
