<!-- Author: Hao Yao <hao.yao@intel.com> -->
<!-- Copyright (c) 2023 Intel Corporation. -->

Intel IPU6 Sensor enabling guidance
===================================

Preparation
-----------

1. Camera sensor specification documents.

2. Recommended register settings from device vendor.

3. Graph settings files (\*.xml) and tuning files (\*.aiqb).

4. Schematics of hardware, including camera module, transfer board and mother board.

5. Intel IPU6 Linux kernel driver source code and a compatible Linux kernel (recommend version > v5.10 and newer is better).

6. Intel IPU6 userspace driver source code, including libcamhal (and binary dependencies installed) and icamerasrc.

Kernel Driver
-------------

### 1. Create a camera sensor driver source code

The MIPI camera sensors are mostly i2c devices so driver source code files are placed in drivers/media/i2c. To make kernel Kbuild system to find it, we need to add corresponding items in Kconfig and Makefile. Add below parts to corresponding files:

drivers/media/i2c/Kconfig:
```config
config VIDEO_<sensor name>
    tristate "<vendor name> <sensor name> sensor support"
    depends on I2C && VIDEO_DEV
    select MEDIA_CONTROLLER
    select VIDEO_V4L2_SUBDEV_API
    select V4L2_FWNODE
    help
      This is a Video4Linux2 sensor driver for the <vendor name>
      <sensor name> camera.
```

drivers/media/i2c/Makefile:
```makefile
obj-$(CONFIG_VIDEO_OV13B10) += ov13b10.o
```

Then when you build the kernel, you can add `CONFIG_VIDEO_<sensor name>=m` to build your driver as a module.

The structure of i2c sensor driver is basically the same. We can take drivers/media/i2c/ov13b10.c as a reference.

### 2. Power on a camera sensor

Usually, a sensor need some resources to work:

- Power supply

- Reference clock source

- A reset pin to control on/off status

- Privacy LED (optional, it can be controlled by v4l2 after kernel v6.3)

- Other optional GPIOs

On IPU6 system, one condition is that those resources can be handled by a power control logic device which HID is `INT3472`. it has a driver in Linux kernel since v5.14. It checks ACPI table to know which GPIOs and clock source a camera sensor uses and create the mapping list in kernel. Then in sensor driver we can use some kernel APIs to get those resources and control them. In this case, we can directly use code like below:

```c
struct ov13b10 {
...
	struct clk *img_clk;
	struct regulator *avdd;
	struct gpio_desc *reset;
...
};

/* This function tries to get power control resources */
static int ov13b10_get_pm_resources(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov13b10 *ov13b = to_ov13b10(sd);
	int ret;

	ov13b->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ov13b->reset))
		return dev_err_probe(dev, PTR_ERR(ov13b->reset),
				     "failed to get reset gpio\n");

	ov13b->img_clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(ov13b->img_clk))
		return dev_err_probe(dev, PTR_ERR(ov13b->img_clk),
				     "failed to get imaging clock\n");

	ov13b->avdd = devm_regulator_get_optional(dev, "avdd");
	if (IS_ERR(ov13b->avdd)) {
		ret = PTR_ERR(ov13b->avdd);
		ov13b->avdd = NULL;
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret,
					     "failed to get avdd regulator\n");
	}

	return 0;
}

static int ov13b10_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov13b10 *ov13b10 = to_ov13b10(sd);

	if (ov13b10->reset)
		gpiod_set_value_cansleep(ov13b10->reset, 1);

	if (ov13b10->avdd)
		regulator_disable(ov13b10->avdd);

	clk_disable_unprepare(ov13b10->img_clk);

	return 0;
}

static int ov13b10_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov13b10 *ov13b10 = to_ov13b10(sd);
	int ret;

	ret = clk_prepare_enable(ov13b10->img_clk);
	if (ret < 0) {
		dev_err(dev, "failed to enable imaging clock: %d", ret);
		return ret;
	}

	if (ov13b10->avdd) {
		ret = regulator_enable(ov13b10->avdd);
		if (ret < 0) {
			dev_err(dev, "failed to enable avdd: %d", ret);
			clk_disable_unprepare(ov13b10->img_clk);
			return ret;
		}
	}

	if (ov13b10->reset) {
		gpiod_set_value_cansleep(ov13b10->reset, 0);
		/* 5ms to wait ready after XSHUTDN assert */
		usleep_range(5000, 5500);
	}

	return 0;
}

/* After this module is loaded, probe function is called
   if this device HID is enumerated by ACPI table. */
static int ov13b10_probe(struct i2c_client *client)
{
...
	ov13b = devm_kzalloc(&client->dev, sizeof(*ov13b), GFP_KERNEL);
	if (!ov13b)
		return -ENOMEM;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&ov13b->sd, client, &ov13b10_subdev_ops);

	ret = ov13b10_get_pm_resources(&client->dev);
	if (ret)
		return ret;

	full_power = acpi_dev_state_d0(&client->dev);
	if (full_power) {
		ov13b10_power_on(&client->dev);
		if (ret) {
			dev_err(&client->dev, "failed to power on\n");
			return ret;
		}
...
	if (full_power)
		pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;
...
error_power_off:
	ov13b10_power_off(&client->dev);

	return ret;
}

static const struct dev_pm_ops ov13b10_pm_ops = {
...
/* This line set power_on/off functions as runtime power management
   callback so when pm_runtime APIs are called, the power_on/off
   functions can actually set the power of camera sensor. */
	SET_RUNTIME_PM_OPS(ov13b10_power_off, ov13b10_power_on, NULL)
};
```

In another case, camera power is controlled by CVF devices and we use `vsc_acquire/release_camera_sensor()` functions to set the power. In this case, we call these APIs in power_on/off functions instead of pull GPIO pins or set clock or regulator.

After the driver can be loaded and probe function can be called, we can directly check the voltages of power pins by multimeter. But we can also check if sensor i2c works instead of checking voltage.

### 3. i2c R/W function

If you know nothing about i2c, please Google to know the common knowledge of i2c and the manual of `i2cdetect` and `i2ctransfer` tools.

We can use `i2cdetect` in bash to detect if the device is on a specific i2c bus and use `i2ctransfer` to directly transfer i2c message on a bus. Usually we try to read the chip ID of a camera sensor to ensure the power pins are ready and i2c works. If the behavior is not expected, we need to check the power management resources by camera sensor's specification document.

Before we can read/write register values in driver, we need to implement the i2c read and write function.

```c
/* Read registers up to 4 at a time */
static int ov13b10_read_reg(struct ov13b10 *ov13b,
			    u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov13b->sd);
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	int ret;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);

	if (len > 4)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

/* Write registers up to 4 at a time */
static int ov13b10_write_reg(struct ov13b10 *ov13b,
			     u16 reg, u32 len, u32 __val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov13b->sd);
	int buf_i, val_i;
	u8 buf[6], *val_p;
	__be32 val;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val = cpu_to_be32(__val);
	val_p = (u8 *)&val;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}
```

One thing to notice is that some chips use 2-byte register address while some use 1-byte. So you need to know exactly what messages the function should send to complete those functions. For example, ov13b10 use 2-byte register address. the `u8 buf[]` stores 2 byte address in `buf[0]` and `buf[1]`, and value should start at `buf[2]`. But for 1-byte address chip, only `buf[0]` is needed to store an address and value should start at `buf[1]`. The total number of messages in calling `i2c_master_send()` should also be changed to `len + 1`.

### 4. Register settings of camera sensors

After camera sensor is powered on, usually we need a set of register settings to write to sensor to initialize it. We can get register settings from vendor or source code from other platforms. It can be integrated to driver like this:

```c
struct ov13b10_reg {
	u16 address;
	u8 val;
};

/* 4208x3120 needs 1120Mbps/lane, 4 lanes */
static const struct ov13b10_reg mipi_data_rate_1120mbps[] = {
	{0x0103, 0x01},
...
	{0x5049, 0xa4},
};

/* Link frequency configs */
static const struct ov13b10_link_freq_config
			link_freq_configs[] = {
	{
		.pixels_per_line = OV13B10_PPL_560MHZ,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mipi_data_rate_1120mbps),
			.regs = mipi_data_rate_1120mbps,
		}
	}
};

static const struct ov13b10_reg mode_4208x3120_regs[] = {
	{0x0305, 0xaf},
...
	{0x5001, 0x0f},
};

/* Mode configs */
static const struct ov13b10_mode supported_modes[] = {
	{
		.width = 4208,
		.height = 3120,
		.vts_def = OV13B10_VTS_30FPS,
		.vts_min = OV13B10_VTS_30FPS,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4208x3120_regs),
			.regs = mode_4208x3120_regs,
		},
		.link_freq_index = OV13B10_LINK_FREQ_INDEX_0,
	},
...
};
```

Different link frequency config needs different initial settings. For different modes, sensor also need different settings and they are applied when format is changed before stream on.

We also need to focus on some key control register address, including:

- Chip ID, to validate if i2c function is OK after calling power_on function.

- Software reset, set right after power on and maybe need to sleep some time to continue register writing.

- Mode select, to set the standby/streaming status of sensor.

- H/V blank, which can control the timing and FPS. V blank usually need a maximum definition.

- Exposure, analog and digital gains. These registers offers interface for userspace to control the exposure of sensor. The ranges of register value is also needed to be considered.

- Apply, update or group write registers. Some sensor need to manually write this registers to let exposure settings take effect.

- Test pattern, which is useful in debugging.

- Page select, which is used by some chips to select register page before actually reading or writing a register.

### 5. Link camera sensor to IPU6

A MIPI camera sensor works as a sub-device of IPU6 in V4L2 framework, so we need to bind the sensor to IPU6 as a sub-device. drivers/media/pci/intel/cio2-bridge.c implemented this function and we can set the sensor's information in it to let IPU6 takes it as its sub-device.

```c
/* cio2-bridge.h */
#define CIO2_SENSOR_CONFIG(_HID, _NR, ...)  \
    ((const struct cio2_sensor_config) {    \
        .hid = (_HID),          \
        .nr_link_freqs = (_NR),     \
        .link_freqs = { __VA_ARGS__ }   \
    })

struct cio2_sensor_config {
    const char *hid;
    const u8 nr_link_freqs;
    const u64 link_freqs[MAX_NUM_LINK_FREQS];
};

/* cio2-bridge.c */
static const struct cio2_sensor_config cio2_supported_sensors[] = {
...
     /* Omnivision ov13b10 */
     CIO2_SENSOR_CONFIG("OVTI13B1", 1, 560000000),
...
};
```

Add the sensor's info to `cio2_supported_sensors` array, then when cio2-bridge find corresponding sensor it binds sensor as a sub-device.

### 6. Validate the raw output with kernel driver only

We can use `media-ctl` to create link between camera sensor and IPU6 ISYS then use `yavta` tool to dump raw image to validate if kernel driver function is OK. We can create the shell script according to reference below:

```bash
#!/bin/bash
export PATH=$PWD:$PWD/jpeg:$PATH
export LD_LIBRARY_PATH=$PWD/jpeg:$LD_LIBRARY_PATH
FRAME_NUM=30
RESOLUTION=1920x1200
FMT=SGRBG10_1X10
VFMT=SGBRG10
WIDTH=$(echo ${RESOLUTION} | awk -F 'x' '{print $1}')
HEIGHT=$(echo ${RESOLUTION} | awk -F 'x' '{print $2}')

media-ctl -r
# 2-0036 is the bus number (2) and address (0x36)
media-ctl -V "\"ov13b10 2-0036\":0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"Intel IPU6 CSI-2 0\":0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"Intel IPU6 CSI-2 0\":1 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"Intel IPU6 CSI2 BE SOC 0\":0 [fmt:$FMT/${RESOLUTION}]"
media-ctl -V "\"Intel IPU6 CSI2 BE SOC 0\":1 [fmt:$FMT/${RESOLUTION}]"
media-ctl -l "\"ov13b10 2-0036\":0 -> \"Intel IPU6 CSI-2 0\":0[1]"
media-ctl -l "\"Intel IPU6 CSI-2 0\":1 -> \"Intel IPU6 CSI2 BE SOC 0\":0[1]"
media-ctl -l "\"Intel IPU6 CSI2 BE SOC 0\":1 -> \"Intel IPU6 BE SOC capture 0\":0[5]"

rm frame-*.bin frame-*.bin.jpg
CAPTURE_DEV=$(media-ctl -e "Intel IPU6 BE SOC capture 0")
./yavta -c${FRAME_NUM} -n5 ${CAPTURE_DEV} -I -s${RESOLUTION} -Fframe-a-#.bin -f $VFMT

for oneframe in `ls *.bin`; do
	echo "Converting $oneframe..."
	jpeg_encoder $oneframe $VFMT $WIDTH $HEIGHT
done
```

---

Userspace Driver
================

IPU6 userspace driver on Linux OS contains different modules, but for new sensor enabling we can focus on libcamhal only. We need to add or change some config files and add a binary tuning file.

### 1. Tuning file (\*.aiqb)

Copy it to `config/linux/<IPU version>` and it will be installed along with other config files.

### 2. Graph settings

It is based on graph descriptor which differs according to IPU version. It contains IPU pipeline settings for different sensor output resolution, format, output format and various different settings. It is placed at `config/linux/<IPU version>/gcss`.

### 3. psys_policy_profiles.xml

It describes IPU PSYS policy for different graph config. For new sensor, we need to check graph settings to know which graph IDs is used for sensor output and fill it in `psys_policy_profiles.xml`.

graph_settings.xml:
```xml
<settings key="8000" id="100000" active_outputs="1" dvs="0" power_factor="0" fps="30" flow="StillsDuringVideo" flip_v="0" flip_h="0" pdaf_type="0">
...
  <sensor vflip="0" hflip="0" mode_id="4208X3120">
            <port_0 format="BG10" width="4208" height="3120" />
  </sensor>
  <csi_be>
    <output format="GR10" width="4208" height="3120" />
    <stream2mmio>
      <input width="4208" height="3120" top="0" left="0" bottom="0" right="0" />
      <output width="4208" height="3120" top="0" left="0" bottom="0" right="0" />
    </stream2mmio>
    <tuning_mode value="0" />
  </csi_be>
  <isa_lb_video>...</isa_lb_video>
  <post_gdc_video>...</post_gdc_video>
</settings>
```

psys_policy_profiles.xml:
```xml
<graph id="100000">
    <!-- op_modes: 0-disabled, 2-coupled. see ia_cipf_css.h in libiacss -->
    <pipe_executor name="ipu6_lb_video"  pgs="isa_lb_video"/>
    <pipe_executor name="ipu6_bb_video"  pgs="post_gdc_video"/>
    <bundle executors="ipu6_lb_video:0,ipu6_bb_video:1"/>
    <enableBundleInSdv name="false"/>

    <pipe_executor name="ipu6_full_still"  pgs="isa_lb_stills,post_gdc_stills"/>
</graph>
```

### 4. libcamhal_profiles.xml

It describes which sensors libcamhal supports. Just add `<sensor-name>-[uf|wf]-<CSI port>` *(UF - User Facing, WF - World Facing)* to `availableSensors` before `imx390`. For sensors with virtual channel support, add it after `imx390`.

### 5. sensor setting xml

It should be placed at `config/linux/<IPU version>/sensors/<sensor-name>-[uf|wf].xml`. Here is an example of ov13b10's config:

ov13b10-wf.xml
```xml
...
<CameraSettings>
    <Sensor name="ov13b10-wf" description="ov13b10 as sensor.">
        <MediaCtlConfig id="0" ConfigMode="AUTO" outputWidth="4208" outputHeight="3120" format="V4L2_PIX_FMT_SGRBG10"> <!-- RAW10 BE capture -->
            <format name="ov13b10 $I2CBUS" pad="0" width="4208" height="3120" format="V4L2_MBUS_FMT_SGRBG10_1X10"/>
            <format name="Intel IPU6 CSI-2 $CSI_PORT" pad="0" width="4208" height="3120" format="V4L2_MBUS_FMT_SGRBG10_1X10"/>
            <format name="Intel IPU6 CSI2 BE SOC $CSI_PORT" pad="0" width="4208" height="3120" format="V4L2_MBUS_FMT_SGRBG10_1X10"/>
            <format name="Intel IPU6 CSI2 BE SOC $CSI_PORT" pad="1" width="4208" height="3120" format="V4L2_MBUS_FMT_SGRBG10_1X10"/>
            <selection name="Intel IPU6 CSI2 BE SOC $CSI_PORT" pad="1" target="V4L2_SEL_TGT_CROP" left="0" top="0" width="4208" height="3120"/>

            <link srcName="ov13b10 $I2CBUS" srcPad="0" sinkName="Intel IPU6 CSI-2 $CSI_PORT" sinkPad="0" enable="true"/>
            <link srcName="Intel IPU6 CSI-2 $CSI_PORT" srcPad="1" sinkName="Intel IPU6 CSI2 BE SOC $CSI_PORT" sinkPad="0" enable="true"/>
            <link srcName="Intel IPU6 CSI2 BE SOC $CSI_PORT" srcPad="1" sinkName="Intel IPU6 BE SOC capture $CSI_PORT" sinkPad="0" enable="true"/>

            <videonode name="Intel IPU6 BE SOC capture $CSI_PORT" videoNodeType="VIDEO_GENERIC"/>
            <videonode name="Intel IPU6 CSI-2 $CSI_PORT" videoNodeType="VIDEO_ISYS_RECEIVER"/>
            <videonode name="ov13b10 $I2CBUS" videoNodeType="VIDEO_PIXEL_ARRAY"/>
        </MediaCtlConfig>

        <StaticMetadata>
            <!-- format,widthxheight,field(none:0,alternate:7),mcId -->
            <supportedStreamConfig value="V4L2_PIX_FMT_NV12,1280x720,0,0,
                                          V4L2_PIX_FMT_NV12,1920x1080,0,0,
                                          V4L2_PIX_FMT_NV12,1280x960,0,0,
                                          V4L2_PIX_FMT_NV12,1600x1200,0,0,
                                          V4L2_PIX_FMT_NV12,640x480,0,0,
                                          V4L2_PIX_FMT_NV12,640x360,0,0,
                                          V4L2_PIX_FMT_NV12,320x240,0,0,
                                          V4L2_PIX_FMT_SGRBG10,4208x3120,0,0,
                                          V4L2_PIX_FMT_NV12,4096x3072,0,0"/>

            <supportedFeatures value="MANUAL_EXPOSURE,
                                      MANUAL_WHITE_BALANCE,
                                      IMAGE_ENHANCEMENT,
                                      NOISE_REDUCTION,
                                      PER_FRAME_CONTROL,
                                      SCENE_MODE"/>
            <fpsRange value="15,15,24,24,15,30,30,30"/>
            <evRange value="-6,6"/>
            <evStep value="1,3"/>
            <supportedAeMode value="AUTO,MANUAL"/>
            <supportedVideoStabilizationModes value="OFF"/>
            <supportedSceneMode value="NORMAL"/>
            <supportedAntibandingMode value="AUTO,50Hz,60Hz,OFF"/>
            <supportedAwbMode value="AUTO,INCANDESCENT,FLUORESCENT,DAYLIGHT,FULL_OVERCAST,PARTLY_OVERCAST,SUNSET,VIDEO_CONFERENCE,MANUAL_CCT_RANGE,MANUAL_WHITE_POINT,MANUAL_GAIN,MANUAL_COLOR_TRANSFORM"/>
            <supportedAfMode value="AUTO,MACRO,CONTINUOUS_VIDEO,CONTINUOUS_PICTURE,OFF"/>

            <!-- 0:FALSE, 1: TRUE -->
            <ae.lockAvailable value="1"/>
            <awb.lockAvailable value="1"/>
            <!-- 0: OFF, 1: AUTO, 2: USE_SCENE_MODE, 3: OFF_KEEP_STATE -->
            <control.availableModes value="0,1"/>
            <!-- 0: DISABLE, 1: FACE_PRIORITY -->
            <control.availableSceneModes value="1"/>
            <control.maxRegions value="1,0,1"/>
            <!-- 0: OFF, 1: SMPLE, 2: FULL -->
            <statistics.info.availableFaceDetectModes value="0,1"/>
            <statistics.info.maxFaceCount value="10"/>

            <sensor.orientation value="0"/>
            <sensor.info.sensitivityRange value="49,800"/>
            <!-- microsecond -->
            <sensor.info.exposureTimeRange value="100,100000"/>  <!-- align with supportedAeExposureTimeRange -->

            <!-- Blow static metadata are for camera HAL V3 -->
            <sensor.info.activeArraySize value="0,0,4208,3120"/>
            <sensor.info.pixelArraySize value="4208x3120"/>
            <!-- 4208x1.12um 3120x1.12um -->
            <sensor.info.physicalSize value="3.67,2.76"/>
            <!-- 0: off, 1: solid color, 2: color bars  -->
            <sensor.availableTestPatternModes value="0,2"/>
            <!-- 0:RGGB, 1:GRBG, 2:GBRG, 3:BGGR, 4:RGB -->
            <sensor.info.colorFilterArrangement value="2"/>
...
            <!-- [width, height, raw-out-size] -->
            <sensor.opaqueRawSize value="4208,3120,100"/>
            <!-- available stream configurations: format: IMPLEMENTATION_DEFINED(34)|YCbCr_420_888:0x23(35)|BLOB(33), widthxheight, type: output(0)|input(1) -->
            <scaler.availableStreamConfigurations value="
                33,4096x3072,0,
                33,1920x1080,0,
                33,1600x1200,0,
                33,1280x960,0,
                33,1280x720,0,
                33,640x480,0,
                33,640x360,0,
                33,320x240,0,
                35,4096x3072,0,
                35,1920x1080,0,
                35,1600x1200,0,
                35,1280x960,0,
                35,1280x720,0,
                35,640x480,0,
                35,640x360,0,
                35,320x240,0,
                34,4096x3072,0,
                34,4096x3072,1,
                34,1920x1080,0,
                34,1600x1200,0,
                34,1280x960,0,
                34,1280x720,0,
                34,640x480,0,
                34,640x360,0,
                34,320x240,0" />
            <!-- minimum frame duration: format: IMPLEMENTATION_DEFINED(34)|YCbCr_420_888:0x23(35)|BLOB(33), widthxheight, duration:(ns) -->
            <scaler.availableMinFrameDurations value="
                33,4096x3072,66666666,
                33,1920x1080,33333333,
                33,1600x1200,33333333,
                33,1280x960,33333333,
                33,1280x720,33333333,
                33,640x480,33333333,
                33,640x360,33333333,
                33,320x240,33333333,
                35,4096x3072,66666666,
                35,1920x1080,33333333,
                35,1600x1200,33333333,
                35,1280x960,33333333,
                35,1280x720,33333333,
                35,640x480,33333333,
                35,640x360,33333333,
                35,320x240,33333333,
                34,4096x3072,66666666,
                34,1920x1080,33333333,
                34,1600x1200,33333333,
                34,1280x960,33333333,
                34,1280x720,33333333,
                34,640x480,33333333,
                34,640x360,33333333,
                34,320x240,33333333" />
            <!-- maximum stall duration: format: IMPLEMENTATION_DEFINED(34)|YCbCr_420_888:0x23(35)|BLOB(33), widthxheight, duration:(ns) -->
            <scaler.availableStallDurations value="
                33,4096x3072,66666666,
                33,1920x1080,33333333,
                33,1600x1200,33333333,
                33,1280x960,33333333,
                33,1280x720,33333333,
                33,640x480,33333333,
                33,640x360,33333333,
                33,320x240,33333333" />

            <jpeg.maxSize value="18874368"/>  <!-- 4096*3072*1.5 -->
            <jpeg.availableThumbnailSizes value="0,0,320,180,320,240"/> <!-- INCREASING ORDER -->
...
        </StaticMetadata>

        <supportedTuningConfig value="NORMAL,VIDEO,ov13b10,
                                      STILL_CAPTURE,VIDEO,ov13b10"/>

        <!-- The lard tags configuration. Every tag should be 4-characters. -->
        <!-- <TuningMode, cmc tag, aiq tag, isp tag, others tag>  -->
        <lardTags value="VIDEO,DFLT,DFLT,DFLT,DFLT"/>

        <supportedISysSizes value="4208x3120"/> <!-- ascending order request -->
        <supportedISysFormat value="V4L2_PIX_FMT_SGRBG10"/>
        <enableAIQ value="true"/>
        <iSysRawFormat value="V4L2_PIX_FMT_SGRBG10"/>
        <maxRawDataNum value="32"/>
        <pSysFormat value="V4L2_PIX_FMT_NV12"/>
        <initialSkipFrame value="0"/>
        <exposureLag value="2"/>
        <gainLag value="2"/>
        <digitalGainLag value="0"/>
        <ltmGainLag value="1"/>
        <yuvColorRangeMode value="full"/> <!-- there are 2 yuv color range mode, like full, reduced. -->
        <pipeSwitchDelayFrame value="60"/>
        <graphSettingsFile value="graph_settings_ov13b10.xml"/>
        <graphSettingsType value="dispersed"/>
        <enablePSysProcessor value="true"/>
        <dvsType value="IMG_TRANS"/>
        <nvmDeviceInfo value="NVM,1680"/>
        <lensName value="dw9714"/>
        <lensHwType value="LENS_VCM_HW"/>
        <testPatternMap value="Off,0,ColorBars,2"/>
        <AlgoRunningRate value="AE,0.2,AWB,0.2"/> <!-- AE running rate is 0.2, AWB running rate is 0.2. -->
        <enableAiqd value = "true"/>
        <useCrlModule value = "false"/>
        <pslOutputMapForRotation value="4096x3072@4096x3072,1920x1080@1920x1080,1600x1200@1600x1200,
                                        1280x960@1280x960,1280x720@1280x720,640x480@1280x960,640x360@1280x720,320x240@640x480"/>

        <maxRequestsInflight value="6"/>
        <faceEngineRunningInterval value="10"/>
        <faceEngineRunningIntervalNoFace value="10"/>
        <faceEngineVendor value="1"/> <!-- pvl face detection:0, google facessd:1 -->
        <faceEngineByIPU value="true"/>
        <psysAlignWithSof value="true"/>
        <psysBundleWithAic value="true"/>
        <skipFrameV4L2Error value="true"/>
        <isISYSCompression value="true"/>
        <isPSACompression value="true"/>
    </Sensor>
</CameraSettings>
```

A `CameraSettings` field can contain multiple `Sensor` fields corresponding to different modules settings, but HAL will only pick the first `Sensor` filed matched. `Sensor` filed contains `MediaCtlConfig`, `StaticMetadata` and other fileds.

`MediaCtlConfig` is basically the same as the media-ctl configs in yavta script, just fill in according to available formats in sensor kernel driver and the link between sensor and IPU. We can have different this field and can be distiguished by `id`.

`StaticMetadata` mostly are related to sensor output format and resolution. Some settings are related to `MediaCtlConfig` IDs.

Other fields need to pay attention:

- `supportedTuningConfig`: fill in the tuning file name we are using.

- `graphSettingsFile`: value is the filename of graph setting xml.

- `exposureLag`, `gainLag` and `digitalGainLag`:
	Usually when we set exposure related registers to sensor, it may takes several frames delay to actually apply them. Mostly they use 2, 2, and 0. We should check these values according to sensor's specification document to make sure AE functioning normally.

- `nvmDeviceInfo` and `lensName`: NVM *(Non-Volatile Memory on sensor module)* name, size and VCM *(Voice Coil Motor)* name. NVM can store some information of sensor module name or calibration data according to sensor module vendor, which may be optional for sensor enabling. VCM can control the lens position for auto focus function.

- `supportPrivacy`: privacy mode support, 0 for no privacy mode support, 1 for CVF privacy mode and 2 for AE-based privacy. For value 2, we also need to set `privacyModeThreshold` and `privacyModeFrameDelay` value.

### 6. Use gstreamer and icamerasrc to test

icamerasrc is a libcamhal-based gstreamer plugin for Intel IPU6 MIPI cameras. Before using icamerasrc, we need to ensure `${GST_PLUGIN_PATH}` is set and `libgsticamerasrc.so` is installed in it. Here is an example commandline:

```bash
# Set GST_PLUGIN_PATH correctly first
sudo -E gst-launch-1.0 icamerasrc device-name=ov13b10-wf af-mode=2 ! video/x-raw,format=NV12,width=1280,height=720 ! videoconvert ! ximagesink
```

For more details you can check `gst-inspect-1.0 icamerasrc`.
