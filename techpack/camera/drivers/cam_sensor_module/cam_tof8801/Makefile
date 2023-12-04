ccflags-y += -Wno-unused-function
ccflags-y += -Itechpack/camera/drivers/cam_sensor_module/cam_tof8801/include
ccflags-y += -Itechpack/camera/drivers/cam_utils
ccflags-y += -Itechpack/camera/drivers/cam_cpas/include
ccflags-y += -Itechpack/camera/drivers/cam_sensor_module/cam_sensor_io
ccflags-y += -Itechpack/camera/drivers/cam_sensor_module/cam_sensor_utils
ccflags-y += -Itechpack/camera/drivers/cam_sensor_module/cam_cci
ccflags-y += -Itechpack/camera/drivers/cam_req_mgr
ccflags-y += -Itechpack/camera/drivers/cam_smmu
ccflags-y += -Itechpack/camera/drivers/cam_core
#ifdef VENDOR_EDIT
obj-$(CONFIG_SENSORS_TMF8801) += tmf8801.o
#else
obj-y += tmf8801.o
#endif
tmf8801-objs := \
	tof8801_pdrv.o \
	tof8801_driver.o \
	tof8801_bootloader.o \
	tof_hex_interpreter.o \
	tof8801_app0.o
