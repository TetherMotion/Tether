Tether Documentation
====================

Welcome to the Tether library documentation. Tether is a modular C++ library for EtherCAT motion control.

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/library_root

.. toctree::
   :maxdepth: 2
   :caption: EtherCAT & HAL

   EXTRACT_ESI
   HAL_PORTING_GUIDE
   PDOModes
   CyclicRealtimeTransport
   FSoECrcResync
   IOProtocol
   IOProtocolWireFormat
   CrossCompiling
   ModelIdentification

.. toctree::
   :maxdepth: 2
   :caption: Motion

   MotionReplanner
   SnapSpaceVelocityProfiler
   motion/Architecture
   motion/GeometryFoundations
   motion/BlendingAlgorithm
   motion/MotionChain
   motion/VelocityProfilerSelection
   motion/CertificationPath
   motion/ToppraDerivation
   motion/AnalyticalTOPPRA
   motion/AlgorithmComparison
   motion/ParetoTimeEnergyOptimal
   motion/WeightedSwitchingStructure
   motion/ImplementationGuide

.. toctree::
   :maxdepth: 2
   :caption: Klipper

   KlipperArchitecture
   KlipperProtocol
   KlipperTerminology
   KlipperGcodeCommands
   KlipperPrinterObjects
   KlipperMoonrakerApi

.. toctree::
   :maxdepth: 2
   :caption: Extrusion

   extrusion/NonNewtonianPressureAdvance
   extrusion/AnalyticalExtrusionCompensation
   extrusion/RheologyModels
   extrusion/DeconvolutionControllers
   extrusion/LPVDeconvolution
   extrusion/FlowAdaptiveTemperatureControl

.. toctree::
   :maxdepth: 2
   :caption: Simulation

   simulation/aerospace_systems
   simulation/biological_systems
   simulation/chaotic_systems
   simulation/chemical_systems
   simulation/delay_systems
   simulation/electrical_systems
   simulation/fluid_systems
   simulation/mechanical_systems
   simulation/rotational_systems
   simulation/system_identification_benchmarks
   simulation/thermal_systems

Overview
--------

The Tether library provides components for:

- Hardware abstraction layer (HAL)
- Control algorithms (PID, motion profiles)
- G-code parsing and trajectory generation
- EtherCAT master and slave implementations
- Motion planning and replanning
- Simulation systems

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
