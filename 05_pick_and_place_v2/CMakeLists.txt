cmake_minimum_required(VERSION 3.5)
project(controllo_ammettenza_skeleton_project CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Trova le dipendenze necessarie
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Franka REQUIRED)
find_package(Threads REQUIRED) # <-- AGGIUNGI QUESTA RIGA

# Trova ZMQ (ZeroMQ) usando pkg-config, un modo comune per trovarlo
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# Aggiungi le directory degli header
include_directories(${ZMQ_INCLUDE_DIRS})

# Definisci l'eseguibile con i suoi file sorgente
add_executable(controllo_ammettenza_pick_and_place
    controllo_ammettenza_pick_and_place.cpp
    examples_common.cpp
    header_capsuleStatiche.cpp
)

# Collega le librerie all'eseguibile
target_link_libraries(controllo_ammettenza_pick_and_place PRIVATE
    Franka::Franka
    Eigen3::Eigen
    ${ZMQ_LIBRARIES}
    Threads::Threads # <-- E AGGIUNGI QUESTA
)