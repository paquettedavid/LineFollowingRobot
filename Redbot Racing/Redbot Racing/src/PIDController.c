#include "PIDController.h"


PIDController PIDControllerCreate(float processVariableSetPoint, float controllerGain,
        float integralTime, float derivativeTime, float minControllerOutput, float maxControllerOutput, float samplingPeriod , float initialControllerOffset) {
    PIDController newController;
    newController.processVariableSetPoint = processVariableSetPoint;
    newController.controllerGain = controllerGain;
    newController.integralTime = integralTime;
    newController.derivativeTime = derivativeTime;
    newController.error = 0;
    newController.previousError = 0;
    newController.errorIntegral = initialControllerOffset;
    newController.controllerOutput = 0;
    newController.minControllerOutput = minControllerOutput;
    newController.maxControllerOutput = maxControllerOutput;
    newController.samplingPeriod = samplingPeriod;
    return newController;
}

void PIDControllerComputeOutput(PIDController *controller, float processVariable) {
    //compute error
    controller->error = controller->processVariableSetPoint - processVariable;
    //compute integral component
    controller->errorIntegral = (1 / controller->integralTime) * controller->samplingPeriod * controller->error + controller->errorIntegral;
    //wind up protection
    controller->errorIntegral = controller->errorIntegral < controller->minControllerOutput*1000 ? controller->minControllerOutput*1000 : controller->errorIntegral;
    controller->errorIntegral = controller->errorIntegral > controller->maxControllerOutput*1000 ? controller->maxControllerOutput*1000 : controller->errorIntegral;
    //compute derivative term
    float derivativeComponent = (controller->derivativeTime*(controller->previousError - controller->error))/controller->samplingPeriod;
    //compute controller output Kc*(e+Ts/Ti*int(e)+Td*d(e)/Ts)
    controller->controllerOutput = controller->controllerGain * (controller->error + controller->errorIntegral + derivativeComponent);
    controller->previousError = controller->error;
    controller->controllerOutput = controller->controllerOutput < controller->minControllerOutput ? controller->minControllerOutput : controller->controllerOutput;
    controller->controllerOutput = controller->controllerOutput > controller->maxControllerOutput ? controller->maxControllerOutput : controller->controllerOutput;
}
