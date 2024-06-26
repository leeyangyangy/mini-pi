make -f ../Makefile.sdk clean
make -f ../Makefile.sdk TFA_DEVICETREE=stm32mp157d-atk TF_A_CONFIG=serialboot ELF_DEBUG_ENABLE='1' all -j12
