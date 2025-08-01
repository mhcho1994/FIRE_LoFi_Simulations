#ifndef RoverExample_Components_Controller__H
#define RoverExample_Components_Controller__H
#include "meta/meta_modelica.h"
#include "util/modelica.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "simulation/simulation_runtime.h"
#ifdef __cplusplus
extern "C" {
#endif


DLLDirection
real_array omc__omcQ_24DER_24RoverExample_24PComponents_24PMadgwickFusionStep(threadData_t *threadData, modelica_real _ax, modelica_real _ay, modelica_real _az, modelica_real _gx, modelica_real _gy, modelica_real _gz, modelica_real _mx, modelica_real _my, modelica_real _mz, real_array _q0, modelica_real _alpha, modelica_real _beta, modelica_real _dt, modelica_real __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERax, modelica_real __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERay, modelica_real __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERaz, modelica_real __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERgx, modelica_real __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERgy, modelica_real __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERgz, modelica_real __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERmx, modelica_real __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERmy, modelica_real __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERmz, real_array __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERq0, modelica_real __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERalpha, modelica_real __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERbeta, modelica_real __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERdt);
DLLDirection
modelica_metatype boxptr__omcQ_24DER_24RoverExample_24PComponents_24PMadgwickFusionStep(threadData_t *threadData, modelica_metatype _ax, modelica_metatype _ay, modelica_metatype _az, modelica_metatype _gx, modelica_metatype _gy, modelica_metatype _gz, modelica_metatype _mx, modelica_metatype _my, modelica_metatype _mz, modelica_metatype _q0, modelica_metatype _alpha, modelica_metatype _beta, modelica_metatype _dt, modelica_metatype __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERax, modelica_metatype __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERay, modelica_metatype __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERaz, modelica_metatype __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERgx, modelica_metatype __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERgy, modelica_metatype __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERgz, modelica_metatype __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERmx, modelica_metatype __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERmy, modelica_metatype __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERmz, modelica_metatype __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERq0, modelica_metatype __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERalpha, modelica_metatype __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERbeta, modelica_metatype __omcQ_24RoverExample_24PComponents_24PMadgwickFusionStep_24funDERdt);
static const MMC_DEFSTRUCTLIT(boxvar_lit__omcQ_24DER_24RoverExample_24PComponents_24PMadgwickFusionStep,2,0) {(void*) boxptr__omcQ_24DER_24RoverExample_24PComponents_24PMadgwickFusionStep,0}};
#define boxvar__omcQ_24DER_24RoverExample_24PComponents_24PMadgwickFusionStep MMC_REFSTRUCTLIT(boxvar_lit__omcQ_24DER_24RoverExample_24PComponents_24PMadgwickFusionStep)


DLLDirection
real_array omc__omcQ_24DER_24RoverExample_24PUtils_24Pquat2eul(threadData_t *threadData, real_array _q, real_array __omcQ_24RoverExample_24PUtils_24Pquat2eul_24funDERq);
DLLDirection
modelica_metatype boxptr__omcQ_24DER_24RoverExample_24PUtils_24Pquat2eul(threadData_t *threadData, modelica_metatype _q, modelica_metatype __omcQ_24RoverExample_24PUtils_24Pquat2eul_24funDERq);
static const MMC_DEFSTRUCTLIT(boxvar_lit__omcQ_24DER_24RoverExample_24PUtils_24Pquat2eul,2,0) {(void*) boxptr__omcQ_24DER_24RoverExample_24PUtils_24Pquat2eul,0}};
#define boxvar__omcQ_24DER_24RoverExample_24PUtils_24Pquat2eul MMC_REFSTRUCTLIT(boxvar_lit__omcQ_24DER_24RoverExample_24PUtils_24Pquat2eul)


DLLDirection
real_array omc__omcQ_24DER_24RoverExample_24PUtils_24Pquatinv(threadData_t *threadData, real_array _q_in, real_array __omcQ_24RoverExample_24PUtils_24Pquatinv_24funDERq_5Fin);
DLLDirection
modelica_metatype boxptr__omcQ_24DER_24RoverExample_24PUtils_24Pquatinv(threadData_t *threadData, modelica_metatype _q_in, modelica_metatype __omcQ_24RoverExample_24PUtils_24Pquatinv_24funDERq_5Fin);
static const MMC_DEFSTRUCTLIT(boxvar_lit__omcQ_24DER_24RoverExample_24PUtils_24Pquatinv,2,0) {(void*) boxptr__omcQ_24DER_24RoverExample_24PUtils_24Pquatinv,0}};
#define boxvar__omcQ_24DER_24RoverExample_24PUtils_24Pquatinv MMC_REFSTRUCTLIT(boxvar_lit__omcQ_24DER_24RoverExample_24PUtils_24Pquatinv)


DLLDirection
real_array omc__omcQ_24DER_24RoverExample_24PUtils_24Pquatprod(threadData_t *threadData, real_array _q_in1, real_array _q_in2, real_array __omcQ_24RoverExample_24PUtils_24Pquatprod_24funDERq_5Fin1, real_array __omcQ_24RoverExample_24PUtils_24Pquatprod_24funDERq_5Fin2);
DLLDirection
modelica_metatype boxptr__omcQ_24DER_24RoverExample_24PUtils_24Pquatprod(threadData_t *threadData, modelica_metatype _q_in1, modelica_metatype _q_in2, modelica_metatype __omcQ_24RoverExample_24PUtils_24Pquatprod_24funDERq_5Fin1, modelica_metatype __omcQ_24RoverExample_24PUtils_24Pquatprod_24funDERq_5Fin2);
static const MMC_DEFSTRUCTLIT(boxvar_lit__omcQ_24DER_24RoverExample_24PUtils_24Pquatprod,2,0) {(void*) boxptr__omcQ_24DER_24RoverExample_24PUtils_24Pquatprod,0}};
#define boxvar__omcQ_24DER_24RoverExample_24PUtils_24Pquatprod MMC_REFSTRUCTLIT(boxvar_lit__omcQ_24DER_24RoverExample_24PUtils_24Pquatprod)


DLLDirection
real_array omc_RoverExample_Components_MadgwickFusionStep(threadData_t *threadData, modelica_real _ax, modelica_real _ay, modelica_real _az, modelica_real _gx, modelica_real _gy, modelica_real _gz, modelica_real _mx, modelica_real _my, modelica_real _mz, real_array _q0, modelica_real _alpha, modelica_real _beta, modelica_real _dt);
DLLDirection
modelica_metatype boxptr_RoverExample_Components_MadgwickFusionStep(threadData_t *threadData, modelica_metatype _ax, modelica_metatype _ay, modelica_metatype _az, modelica_metatype _gx, modelica_metatype _gy, modelica_metatype _gz, modelica_metatype _mx, modelica_metatype _my, modelica_metatype _mz, modelica_metatype _q0, modelica_metatype _alpha, modelica_metatype _beta, modelica_metatype _dt);
static const MMC_DEFSTRUCTLIT(boxvar_lit_RoverExample_Components_MadgwickFusionStep,2,0) {(void*) boxptr_RoverExample_Components_MadgwickFusionStep,0}};
#define boxvar_RoverExample_Components_MadgwickFusionStep MMC_REFSTRUCTLIT(boxvar_lit_RoverExample_Components_MadgwickFusionStep)


DLLDirection
real_array omc_RoverExample_Utils_quat2eul(threadData_t *threadData, real_array _q);
DLLDirection
modelica_metatype boxptr_RoverExample_Utils_quat2eul(threadData_t *threadData, modelica_metatype _q);
static const MMC_DEFSTRUCTLIT(boxvar_lit_RoverExample_Utils_quat2eul,2,0) {(void*) boxptr_RoverExample_Utils_quat2eul,0}};
#define boxvar_RoverExample_Utils_quat2eul MMC_REFSTRUCTLIT(boxvar_lit_RoverExample_Utils_quat2eul)


DLLDirection
real_array omc_RoverExample_Utils_quatinv(threadData_t *threadData, real_array _q_in);
DLLDirection
modelica_metatype boxptr_RoverExample_Utils_quatinv(threadData_t *threadData, modelica_metatype _q_in);
static const MMC_DEFSTRUCTLIT(boxvar_lit_RoverExample_Utils_quatinv,2,0) {(void*) boxptr_RoverExample_Utils_quatinv,0}};
#define boxvar_RoverExample_Utils_quatinv MMC_REFSTRUCTLIT(boxvar_lit_RoverExample_Utils_quatinv)


DLLDirection
real_array omc_RoverExample_Utils_quatprod(threadData_t *threadData, real_array _q_in1, real_array _q_in2);
DLLDirection
modelica_metatype boxptr_RoverExample_Utils_quatprod(threadData_t *threadData, modelica_metatype _q_in1, modelica_metatype _q_in2);
static const MMC_DEFSTRUCTLIT(boxvar_lit_RoverExample_Utils_quatprod,2,0) {(void*) boxptr_RoverExample_Utils_quatprod,0}};
#define boxvar_RoverExample_Utils_quatprod MMC_REFSTRUCTLIT(boxvar_lit_RoverExample_Utils_quatprod)
#include "RoverExample.Components.Controller_model.h"


#ifdef __cplusplus
}
#endif
#endif

