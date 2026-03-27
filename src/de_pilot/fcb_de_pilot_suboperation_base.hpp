#ifndef FCB_DE_PILOT_SUBOPERATION_BASE_H_
#define FCB_DE_PILOT_SUBOPERATION_BASE_H_

#include "fcb_de_pilot_task_base.hpp"
#include <string>

namespace de {
namespace fcb {
namespace depilot {

class CDEPilotSubOperationBase : public CDEPilotTaskBase {
public:
    virtual ~CDEPilotSubOperationBase() = default;

    // Optional context payload for events (e.g., yaw angle/rate)
    virtual std::string getEventContext() const { return std::string("{}"); }

    // Helper to transition task state consistently
    void transitionTo(DEPILOT_TASK_STATE new_state);

protected:
    CDEPilotSubOperationBase(const std::string& task_name)
        : m_task_name(task_name) {}

    std::string m_task_name;
};

inline void CDEPilotSubOperationBase::transitionTo(DEPILOT_TASK_STATE new_state) {
    m_task_state = new_state;
    // Manager will observe state changes and emit events
}

} // namespace depilot
} // namespace fcb
} // namespace de

#endif // FCB_DE_PILOT_SUBOPERATION_BASE_H_
