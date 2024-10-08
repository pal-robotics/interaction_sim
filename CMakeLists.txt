cmake_minimum_required(VERSION 3.8)
project(interaction_sim)

find_package(ament_cmake REQUIRED)

# register node configurations
ament_index_register_resource("pal_configuration.hri_face_detect" CONTENT config/20-interaction_sim.yml)
ament_index_register_resource("pal_configuration.expressive_eyes" CONTENT config/20-interaction_sim.yml)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
