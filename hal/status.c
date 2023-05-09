#include "sensor_fusion.h"
#include "status.h"

static void   ssSetStatus 			(struct StatusSubsystem *pStatus, fusion_status_t status){}
static void   ssUpdateStatus 		(struct StatusSubsystem *pStatus){}

void initializeStatusSubsystem(StatusSubsystem *pStatus){
    pStatus->previous = OFF;
    pStatus->status = OFF;
    pStatus->next = OFF;
    pStatus->set = ssSetStatus;
    pStatus->queue = ssSetStatus;
    pStatus->update = ssUpdateStatus;
    pStatus->test = ssUpdateStatus;
}
