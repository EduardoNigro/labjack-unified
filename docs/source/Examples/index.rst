
Examples
========

These are some examples that can be executed with any **LabJack** (**U3**, **U6**,
or **T7**). The table below shows the hardware that is required to run them. Four
of the examples only need a pair of wires. Two examples require a **T7** device.

The idea of the examples is to go beyond just using the LabJack device, by showing
some execution loop and sampling techiques.

================================================================  =======================================================
Example                                                           Hardware Requirements
================================================================  =======================================================
:doc:`Analog I/O <Analog_IO>`                                     Two wires
:doc:`Digital I/O <Digital_IO>`                                   Two wires
:doc:`Streaming <Streaming>`                                      Two wires
:doc:`Incremental Encoder <Incremental_Encoder>`                  Quadrature encoder wit A-B phases
:doc:`Absolute Encoder <Absolute_Encoder>`                        Quadrature encoder wit A-B-Z phases
:doc:`Electric Motor <Electric_Motor>`                            12V power supply, 12V DC motor, H-Bridge
:doc:`Electric Motor with Encoder <Electric_Motor_with_Encoder>`  12V power supply, 12V DC motor with encoder, H-Bridge
:doc:`T7 Streaming (internal clock) <T7_Streaming_Internal>`      Two wires
:doc:`T7 Streaming (external clock) <T7_Streaming_External>`      12V power supply, 12V DC motor, H-Bridge, A-B-Z encoder
================================================================  =======================================================


.. note::
   For best graphic display of the results, consider running them in an interactive
   Jupyter notebook session (instead of a plain old terminal window).


.. toctree::
   :hidden:

   Analog_IO
   Digital_IO
   Streaming
   Incremental_Encoder
   Absolute_Encoder
   Electric_Motor
   Electric_Motor_with_Encoder
   T7_Streaming_Internal
   T7_Streaming_External
