idf_component_register(SRCS "MPU6050.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES I2Cdev
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC MPU6050_INCLUDE_DMP_MOTIONAPPS20)