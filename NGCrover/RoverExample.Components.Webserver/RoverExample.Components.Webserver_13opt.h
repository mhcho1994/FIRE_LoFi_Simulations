#if defined(__cplusplus)
  extern "C" {
#endif
  int RoverExample_Components_Webserver_mayer(DATA* data, modelica_real** res, short*);
  int RoverExample_Components_Webserver_lagrange(DATA* data, modelica_real** res, short *, short *);
  int RoverExample_Components_Webserver_pickUpBoundsForInputsInOptimization(DATA* data, modelica_real* min, modelica_real* max, modelica_real*nominal, modelica_boolean *useNominal, char ** name, modelica_real * start, modelica_real * startTimeOpt);
  int RoverExample_Components_Webserver_setInputData(DATA *data, const modelica_boolean file);
  int RoverExample_Components_Webserver_getTimeGrid(DATA *data, modelica_integer * nsi, modelica_real**t);
#if defined(__cplusplus)
}
#endif