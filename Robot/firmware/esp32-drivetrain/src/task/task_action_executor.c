#include "task/task_action_executor.h"

#include <stddef.h>

bool task_action_executor_is_valid(const TaskActionExecutor *executor) {
    return executor != NULL && executor->start != NULL &&
           executor->update != NULL && executor->cancel != NULL;
}
