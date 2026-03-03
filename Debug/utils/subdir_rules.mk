################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
utils/%.obj: ../utils/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/ASUS/ti_workspace/Final_project_anti_theft" --include_path="C:/ti/CC3200SDK_1.5.0/cc3200-sdk/simplelink_extlib/provisioninglib" --include_path="C:/ti/CC3200SDK_1.5.0/cc3200-sdk/example/common" --include_path="C:/ti/CC3200SDK_1.5.0/cc3200-sdk/inc" --include_path="C:/ti/CC3200SDK_1.5.0/cc3200-sdk/driverlib" --include_path="C:/ti/CC3200SDK_1.5.0/cc3200-sdk/simplelink/include" --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --define=cc3200 -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="utils/$(basename $(<F)).d_raw" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


