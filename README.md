# roboflex.realsense

Support for the Realsense D435 camera. We really just wrap the sdk and encode data into messages. It might work with other D4** cameras - YMMV.

## System Dependencies

Sort of: none. We use cmake to build. If find_package(realsense2) fails (meaning cmake can't find some system-installed version of realsense2), we fetch and build it. 

## Install

    pip install roboflex.realsense

## Import

    import roboflex.realsense as rr

## Nodes

There is only one: **RealsenseSensor**

You can use a class method to discover the serial numbers of all the cameras currently connected to your computer:

    rr.RealsenseSensor.get_connected_device_serial_numbers()

There are two ways to instantiate realsense sensors:

1. Via the constructor - you must know the serial number. See below for documentation of the Config type.

        sensor = rr.RealsenseSensor(
            serial_number: str,
            config: rr.Config,
            name: str = "RealsenseSensor",
        )

2. Via a class method - if you have a single realsense attached,
this is the easiest way - you don't even have to know the serial number.

        sensor = rr.RealsenseSensor.get_one_sensor(
            config: rr.Config,
            name: str = "RealsenseSensor",
        )

Use it like so:

    # must be started!
    sensor.start()

    # You can get the serial number:
    sensor.serial_number -> str

    # You can get the configuration back:
    sensor.config -> rr.Config

    # You can get the two camera-k matrices:
    sensor.depth_camera_k -> np.array
    sensor.color_camera_k -> np.array

    # You can get various device values:
    sensor.width_pixels_color -> int
    sensor.height_pixels_color -> int
    sensor.width_pixels_depth -> int
    sensor.height_pixels_depth -> int
    sensor.fps_color -> int
    sensor.fps_depth -> int

    # You can manually trigger a message event:
    # (for the most part you won't need this - only
    # if you have special need to control the message
    # production), and don't want to use "start".
    sensor.produce()


## Messages

RealsenseSensor produces a single message type: **RealsenseFrameset**.

    from roboflex.realsense import RealsenseFrameset

API:

    # the timestamp just before reading from device
    message.t0 -> Float

    # the timestamp just after reading from device
    message.t1 -> Float

    # numpy array of shape=(height, width, 3), dtype=uint8
    message.rgb -> np.ndarray

    # numpy array of shape=(height, width), dtype=uint16
    message.depth -> np.ndarray

    # which camera (if any) the frame is aligned to
    message.aligned_to -> rr.CameraType

    # the serial number of the camera that produced this frameset
    message.serial_number -> str

    # the color camera k of the camera that produced this frameset
    message.camera_k_rgb -> numpy array of (3, 3)

    # the depth camera k of the camera that produced this frameset
    message.camera_k_depth -> numpy array of (3, 3)

    # the index, from the device, of this frameset
    message.frame_number -> int

    # the timestamp, from the device, of this frameset, in epoch seconds
    message.timestamp -> float

DYNOFLEX:

    d = DynoFlex.from_msg(message)

    # the timestamp just before reading from device
    d["t0"] -> Double

    # the timestamp just after reading from device
    d["t1"] -> Double

    # numpy array of shape=(height, width, 3), dtype=uint8
    d["rgb"] -> np.ndarray

    # numpy array of shape=(height, width), dtype=uint16
    d["depth"] -> np.ndarray

    # which camera (if any) the frame is aligned to
    d["aligned_to"] -> rr.CameraType

    # the serial number of the camera that produced this frameset
    d["serial_number"] -> str

    # the color camera k of the camera that produced this frameset
    d["camera_k_rgb"] -> numpy array of (3, 3)

    # the depth camera k of the camera that produced this frameset
    d["camera_k_depth"] -> numpy array of (3, 3)

    # the index, from the device, of this frameset
    d["n"] -> int

    # the timestamp, from the device, of this frameset, in epoch seconds
    d["t"] -> float

## Other

Some types used for configuration

Some enums:

    CameraType:
        NONE,
        RGB,
        DEPTH

    D400VisualPreset:
        CUSTOM,
        DEFAULT,
        HAND,
        HIGH_ACCURACY,
        HIGH_DENSITY,
        MEDIUM_DENSITY

TemporalFilterParameters:

    f = rr.TemporalFilterParameters(
        alpha: float = 0.4,
        delta: float = 20.0,
        persistance_control: int = 7,
    )

    f.alpha -> float
    f.delta -> float
    f.persistance_control -> int

Config:

    # all paramters are optional - defaults shown below
    c = rr.Config(
        align_to: rr.CameraType = rr.CameraType.NONE,
        prioritize_ae: Bool = False,
        rgb_settings: Dict[str, int] = {
            "fps": 0,
            "width": 0,
            "height": 0,
        },
        depth_settings: Dict[str, int] = {
            "fps": 0,
            "width": 0,
            "height": 0,
        },
        depth_visual_preset: rr.D400VisualPreset = rr.D400VisualPreset.DEFAULT,
        temporal_filter_parameters: rr.TemporalFilterParameters = None,
        hole_filling_mode: Optional[int] = None,
        decimation_filter: Optional[int] = None,
    )

    # which camera the frames will be aligned to (if any)
    c.align_to -> rr.CameraType

    # When `true`, allows fps to drop in order to better expose
    # frames, such as in dimly lit environments
    c.prioritize_ae -> Bool

    # fps, height, and width of the color camera
    c.rgb_settings -> Dict[str, int]

    # fps, height, and width of the depth camera
    c.depth_settings -> Dict[str, int]

    # the depth camera visual preset
    c.depth_visual_preset -> rr.D400VisualPreset

    # the temporal filter, if any - can be None
    c.temporal_filter_parameters -> rr.TemporalFilterParameters

    # the 'hole filling mode', if any - can be None
    # 0: fill_from_left
    # 1: farest_from_around
    # 2: nearest_from_around
    c.hole_filling_mode -> int

    # NOTE!
    The realsense camera supports many more settings, such as laser power, etc. If there's something you want supported that's not
    here, just let me know.
