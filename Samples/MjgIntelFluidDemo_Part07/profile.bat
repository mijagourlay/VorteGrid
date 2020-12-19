@rem profile: Run the profiling benchmark, varying the number of threads

set NUMBER_OF_PROCESSORS_ORIG=%NUMBER_OF_PROCESSORS%
set NUMBER_OF_PROCESSORS=1
VorteGridPTbb
set NUMBER_OF_PROCESSORS=2
VorteGridPTbb
set NUMBER_OF_PROCESSORS=4
VorteGridPTbb
set NUMBER_OF_PROCESSORS=8
VorteGridPTbb
set NUMBER_OF_PROCESSORS=%NUMBER_OF_PROCESSORS_ORIG%

pause