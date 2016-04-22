#ifndef PIDCONTROLLER
#define PIDCONTROLLER

typedef struct PIDController {
    float processVariableSetPoint;
    float controllerGain;
    float integralTime;
    float derivativeTime;
    float error;
    float previousError;
    float errorIntegral;
    float minControllerOutput;
    float maxControllerOutput;
    float samplingPeriod;
    float controllerOutput;
} PIDController;

PIDController PIDControllerCreate(float processVariableSetPoint, float controllerGain,
        float integralTime, float derivativeTime float minControllerOutput, float maxControllerOutput, float samplingPeriod, float initialControllerOffset);

void PIDControllerComputeOutput(PIDController *controller, float processVariable);

#endif
