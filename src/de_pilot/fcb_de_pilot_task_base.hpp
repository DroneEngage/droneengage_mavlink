#ifndef FCB_DE_PILOT_TASK_BASE_H_
#define FCB_DE_PILOT_TASK_BASE_H_

#include <cstdint>
#include <string>
#include "fcb_de_pilot_lifecycle_base.hpp"

namespace de {
namespace fcb {
namespace depilot {

enum class DEPILOT_TASK_STATE {
    REQUESTED = 0,
    ACTIVE = 1,
    COMPLETED = 2,
    CANCELLED = 3,
    FAILED = 4
};

class CDEPilotTaskBase : public CDEPilotLifecycleBase {
public:
    virtual ~CDEPilotTaskBase() = default;

    // Task-specific interface
    virtual DEPILOT_TASK_STATE getTaskState() const = 0;
    virtual void setTaskState(DEPILOT_TASK_STATE state) = 0;
    virtual std::string getEventContext() const { return std::string("{}"); }

protected:
    // Common task state
    DEPILOT_TASK_STATE m_task_state = DEPILOT_TASK_STATE::REQUESTED;
};

} // namespace depilot
} // namespace fcb
} // namespace de

#endif // FCB_DE_PILOT_TASK_BASE_H_
