roscpp_dep = dependency('roscpp', required : true)
rostime_dep = dependency('rostime', required : true)
rosconsole_dep = dependency('rosconsole', required : true)
roscpp_serialization_dep = dependency('roscpp_serialization', required : true)
image_transport_dep = dependency('image_transport', required : true)

opencv_dep = declare_dependency(
    include_directories: include_directories('/usr/include/boost/compute/interop/opencv'),
    dependencies: dependency('opencv4', required: true)  # Ensure OpenCV4 is installed
)

rpicam_still = executable('rpicam-still', files('rpicam_still.cpp'),
                          include_directories : include_directories('..'),
                          dependencies: [libcamera_dep, boost_dep],
                          link_with : rpicam_app,
                          install : true)

rpicam_vid = executable('rpicam-vid', files('rpicam_vid.cpp'),
                        include_directories : include_directories('..'),
                        dependencies: [libcamera_dep, boost_dep],
                        link_with : rpicam_app,
                        install : true)

rpicam_hello = executable('rpicam-hello', files('rpicam_hello.cpp'),
                          include_directories : include_directories('..'),
                          dependencies: [
                                libcamera_dep,
                                roscpp_dep,
                                rostime_dep,
                                rosconsole_dep,
                                roscpp_serialization_dep,
                                image_transport_dep,
                                opencv_dep
                            ],
                          link_with : rpicam_app,
                          install : true)

rpicam_raw = executable('rpicam-raw', files('rpicam_raw.cpp'),
                        include_directories : include_directories('..'),
                        dependencies: [libcamera_dep, boost_dep],
                        link_with : rpicam_app,
                        install : true)

rpicam_jpeg = executable('rpicam-jpeg', files('rpicam_jpeg.cpp'),
                         include_directories : include_directories('..'),
                         dependencies: [libcamera_dep, boost_dep],
                         link_with : rpicam_app,
                         install : true)

# Install symlinks to the old app names for legacy purposes.
install_symlink('libcamera-still',
                install_dir: get_option('bindir'),
                pointing_to: 'rpicam-still')

install_symlink('libcamera-vid',
                install_dir: get_option('bindir'),
                pointing_to: 'rpicam-vid')

install_symlink('libcamera-hello',
                install_dir: get_option('bindir'),
                pointing_to: 'rpicam-hello')

install_symlink('libcamera-raw',
                install_dir: get_option('bindir'),
                pointing_to: 'rpicam-raw')

install_symlink('libcamera-jpeg',
                install_dir: get_option('bindir'),
                pointing_to: 'rpicam-jpeg')    

if enable_tflite
    rpicam_detect = executable('rpicam-detect', files('rpicam_detect.cpp'),
                               include_directories : include_directories('..'),
                               dependencies: [libcamera_dep, boost_dep],
                               link_with : rpicam_app,
                               install : true)

    install_symlink('libcamera-detect',
                    install_dir: get_option('bindir'),
                    pointing_to: 'rpicam-detect')
endif
