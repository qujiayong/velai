/*
by Anson from KittenBot HK

Velai designed for Micro:bit
*/

//% color="#32A852" weight=10 icon="\uf0e7"
//% groups='["Actuator", "Ultrasonic/Mic", "Linefollower", "Bumper", "Environment", "Color/Gesture", "Mp3", "RFID", "RGB", "InfraTemp", "Led segment"]'

namespace Velai {

    const PCA9685_ADDRESS = 0x40
    const MODE1 = 0x00
    const MODE2 = 0x01
    const SUBADR1 = 0x02
    const SUBADR2 = 0x03
    const SUBADR3 = 0x04
    const PRESCALE = 0xFE
    const LED0_ON_L = 0x06
    const LED0_ON_H = 0x07
    const LED0_OFF_L = 0x08
    const LED0_OFF_H = 0x09
    const ALL_LED_ON_L = 0xFA
    const ALL_LED_ON_H = 0xFB
    const ALL_LED_OFF_L = 0xFC
    const ALL_LED_OFF_H = 0xFD

    const KC_ADDR = 0x6D
    const KC_VERSION = 0x00
    const KC_MODE = 1
    const KC_READCOLOR = 21
    const KC_READCOLORRAW = 23
    const KC_LEDPWM = 24
    const KC_LEDONOFF = 25
    const KC_LEDBIT = 26
    const KC_PROXIMITY = 31
    const KC_GESTURE = 41

    const RFID_ADDR = 0x6B
    const RFID_VERSION = 0x00
    const RFID_READCMD = 0x01
    const RFID_READOUT = 0x02
    const RFID_WRITE = 0x03
    const RFID_STOP = 0x04
    const RFID_STATUS = 0x05
    const RFID_UUID = 0x06
    
    //STEPPER
    const STP_CHA_L = 2047
    const STP_CHA_H = 4095

    const STP_CHB_L = 1
    const STP_CHB_H = 2047

    const STP_CHC_L = 1023
    const STP_CHC_H = 3071

    const STP_CHD_L = 3071
    const STP_CHD_H = 1023

    const RGB_PIX = 64; // multiple panel support moved to neomatrix
    const RGB_M = 8;

    const COMMAND_I2C_ADDRESS = 0x24
    const DISPLAY_I2C_ADDRESS = 0x34
    const _SEG = [0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71];

    let _intensity = 3
    let dbuf = [0, 0, 0, 0]
    //let tracerport=[null,null]
    const FontNum = [
        0xff81ff,
        0x0000ff,
        0x8f89f9,
        0xff8989,
        0xff080f,
        0xf9898f,
        0xf989ff,
        0xff0101,
        0xff89ff,
        0xff898f
    ]

    enum RfidStat {
        IDLE = 0,
        SELECTED = 1,
        READ_PENDING = 2,
        READ_SUCC = 3,
        WRITE_SUCC = 4
    }

    export enum PrevNext {
        //% block=play
        Play = 0xaa,
        //% block=stop
        Stop = 0xab,
        //% block=next
        Next = 0xac,
        //% block=prev
        Prev = 0xad
    }

    export enum GCLed {
        All = 0,
        LED1 = 1,
        LED2 = 2,
        LED3 = 3,
        LED4 = 4
    }

    export enum GCOnOff {
        On = 1,
        Off = 0
    }

    export enum DHT11Type {
        //% block=temperature(°C)
        TemperatureC = 0,
        //% block=temperature(°F)
        TemperatureF = 1,
        //% block=humidity
        Humidity = 2
    }

    export enum RfidSector {
        S1 = 1,
        S2 = 2,
        S3 = 3,
        S4 = 4,
        S5 = 5,
        S6 = 6,
        S7 = 7,
        S8 = 8,
        S9 = 9,
        S10 = 10,
        S11 = 11,
        S12 = 12,
        S13 = 13,
        S14 = 14,
        S15 = 15
    }

    export enum RfidBlock {
        B0 = 0,
        B1 = 1,
        B2 = 2
    }

    export enum TracerEvent {
        InLine = DAL.MICROBIT_BUTTON_EVT_DOWN,
        OutLine = DAL.MICROBIT_BUTTON_EVT_UP
    }

    export enum GCMode {
        //% block=colorsensor
        ColorSensor = 0x1,
        //% block=proximity
        Proximity = 0x2,
        //% block=gesture
        Gesture = 0x3,
        //% block=led
        LED = 0x4
    }

    export enum GCRgb {
        //% block=brightness
        Brightness = 0,
        //% block=red
        Red = 1,
        //% block=green
        Green = 2,
        //% block=blue
        Blue = 3
    }
    export enum NeoPixelColors {
        //% block=red
        Red = 0xFF0000,
        //% block=orange
        Orange = 0xFFA500,
        //% block=yellow
        Yellow = 0xFFFF00,
        //% block=green
        Green = 0x00FF00,
        //% block=blue
        Blue = 0x0000FF,
        //% block=indigo
        Indigo = 0x4b0082,
        //% block=violet
        Violet = 0x8a2be2,
        //% block=purple
        Purple = 0xFF00FF,
        //% block=white
        White = 0xFFFFFF,
        //% block=black
        Black = 0x000000
    }
	
    export enum Servos {
        S1 = 0x01,
        S2 = 0x02,
        S3 = 0x03,
        S4 = 0x04,
        S5 = 0x05,
        S6 = 0x06,
        S7 = 0x07,
        S8 = 0x08
    }

    export enum Motors {
        M1A = 0x1,
        M1B = 0x2,
        M2A = 0x3,
        M2B = 0x4
    }

    export enum Steppers {
        M1 = 0x1,
        M2 = 0x2
    }
	
    export enum Turns {
        //% blockId="T1B4" block="1/4"
        T1B4 = 90,
        //% blockId="T1B2" block="1/2"
        T1B2 = 180,
        //% blockId="T1B0" block="1"
        T1B0 = 360,
        //% blockId="T2B0" block="2"
        T2B0 = 720,
        //% blockId="T3B0" block="3"
        T3B0 = 1080,
        //% blockId="T4B0" block="4"
        T4B0 = 1440,
        //% blockId="T5B0" block="5"
        T5B0 = 1800
    }
	
    let initialized = false
    let distanceBuf = 0;
    let dht11Temp = -1;
    let dht11Humi = -1;

    type EvtAct = () => void;
    let onRfidPresent: EvtAct = null;
    let rgbBuf: Buffer = pins.createBuffer(RGB_PIX * 3);
    let rgbPin: DigitalPin;
    let rgbBright: number = 30;	

    function setBufferRGB(offset: number, red: number, green: number, blue: number): void {
        rgbBuf[offset + 0] = green;
        rgbBuf[offset + 1] = red;
        rgbBuf[offset + 2] = blue;
    }

    //% shim=powerbrick::dht11Update
    function dht11Update(pin: number): number {
        return 999;
    }

    /**
     * send command to display
     * @param is command, eg: 0
     */
    function segCmd(c: number) {
        pins.i2cWriteNumber(COMMAND_I2C_ADDRESS, c, NumberFormat.Int8BE)
    }

    /**
     * send data to display
     * @param is data, eg: 0
     */
    function segDat(bit: number, d: number) {
        pins.i2cWriteNumber(DISPLAY_I2C_ADDRESS + (bit % 4), d, NumberFormat.Int8BE)
    }

    function i2cwrite(addr: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2ccmd(addr: number, value: number) {
        let buf = pins.createBuffer(1)
        buf[0] = value
        pins.i2cWriteBuffer(addr, buf)
    }


    function i2cread(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function initPCA9685(): void {
        i2cwrite(PCA9685_ADDRESS, MODE1, 0x00)
        setFreq(50);
        for (let idx = 0; idx < 16; idx++) {
            setPwm(idx, 0, 0);
        }
        initialized = true
    }
	
  function setFreq(freq: number): void {
        // Constrain the frequency
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
        let oldmode = i2cread(PCA9685_ADDRESS, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2cwrite(PCA9685_ADDRESS, MODE1, newmode); // go to sleep
        i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale); // set the prescaler
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);
        control.waitMicros(5000);
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15)
            return;
        //serial.writeValue("ch", channel)
        //serial.writeValue("on", on)
        //serial.writeValue("off", off)

        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on >> 8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf);
    }
	function setStepper(index: number, dir: boolean): void {
        if (index == 1) {
            if (dir) {
                setPwm(0, STP_CHA_L, STP_CHA_H);
                setPwm(2, STP_CHB_L, STP_CHB_H);
                setPwm(1, STP_CHC_L, STP_CHC_H);
                setPwm(3, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(3, STP_CHA_L, STP_CHA_H);
                setPwm(1, STP_CHB_L, STP_CHB_H);
                setPwm(2, STP_CHC_L, STP_CHC_H);
                setPwm(0, STP_CHD_L, STP_CHD_H);
            }
        } else {
            if (dir) {
                setPwm(4, STP_CHA_L, STP_CHA_H);
                setPwm(6, STP_CHB_L, STP_CHB_H);
                setPwm(5, STP_CHC_L, STP_CHC_H);
                setPwm(7, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(7, STP_CHA_L, STP_CHA_H);
                setPwm(5, STP_CHB_L, STP_CHB_H);
                setPwm(6, STP_CHC_L, STP_CHC_H);
                setPwm(4, STP_CHD_L, STP_CHD_H);
            }
        }
    }

    function stopMotor(index: number) {
        setPwm((index - 1) * 2, 0, 0);
        setPwm((index - 1) * 2 + 1, 0, 0);
    }

    //% blockId=robotbit_servo block="Servo|%index|degree %degree"	
    //% group="Actuator" weight=100
    //% degree.min=0 degree.max=180
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function Servo(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        // 50hz: 20,000 us
        let v_us = (degree * 1800 / 180 + 600) // 0.6 ~ 2.4
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }
	
    /**
     * Geek Servo
     * @param index Servo Channel; eg: S1
     * @param degree [-45-225] degree of servo; eg: -45, 90, 225
    */
    //% blockId=robotbit_gservo block="Geek Servo|%index|degree %degree"
    //% group="Actuator" weight=99
    //% degree.min=-45 degree.max=225
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function GeekServo(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        // 50hz: 20,000 us
        let v_us = ((degree - 90) * 20 / 3 + 1500) // 0.6 ~ 2.4
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }
     /**
     * GeekServo2KG
     * @param index Servo Channel; eg: S1
     * @param degree [0-360] degree of servo; eg: 0, 180, 360
    */
    //% blockId=robotbit_gservo2kg block="GeekServo2KG|%index|degree %degree"
    //% group="Actuator" weight=98
    //% blockGap=50
    //% degree.min=0 degree.max=360
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function GeekServo2KG(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        // 50hz: 20,000 us
        //let v_us = (degree * 2000 / 360 + 500)  0.5 ~ 2.5
        let v_us = (Math.floor((degree) * 2000 / 350) + 500) //fixed
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }
	
    //% blockId=robotbit_stepper_degree block="Stepper 28BYJ-48|%index|degree %degree"
    //% group="Actuator" weight=90
    export function StepperDegree(index: Steppers, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        setStepper(index, degree > 0);
        degree = Math.abs(degree);
        basic.pause(10240 * degree / 360);
        MotorStopAll()
    }


    //% blockId=robotbit_stepper_dual block="Dual Stepper(Degree) |M1 %degree1| M2 %degree2"
    //% group="Actuator" weight=89
    export function StepperDual(degree1: number, degree2: number): void {
        if (!initialized) {
            initPCA9685()
        }
        setStepper(1, degree1 > 0);
        setStepper(2, degree2 > 0);
        degree1 = Math.abs(degree1);
        degree2 = Math.abs(degree2);
        basic.pause(10240 * Math.min(degree1, degree2) / 360);
        if (degree1 > degree2) {
            stopMotor(3); stopMotor(4);
            basic.pause(10240 * (degree1 - degree2) / 360);
        } else {
            stopMotor(1); stopMotor(2);
            basic.pause(10240 * (degree2 - degree1) / 360);
        }

        MotorStopAll()
    }
	
	
  /**
     * Stepper Car move forward
     * @param distance Distance to move in cm; eg: 10, 20
     * @param diameter diameter of wheel in mm; eg: 48
    */
    //% blockId=robotbit_stpcar_move block="Car Forward|Distance(cm) %distance|Wheel Diameter(mm) %diameter"
    //% group="Actuator" weight=88
    export function StpCarMove(distance: number, diameter: number): void {
        if (!initialized) {
            initPCA9685()
        }
        let delay = 10240 * 10 * distance / 3 / diameter; // use 3 instead of pi
        setStepper(1, delay > 0);
        setStepper(2, delay > 0);
        delay = Math.abs(delay);
        basic.pause(delay);
        MotorStopAll()
    }
	

    /**
     * Stepper Car turn by degree
     * @param turn Degree to turn; eg: 90, 180, 360
     * @param diameter diameter of wheel in mm; eg: 48
     * @param track track width of car; eg: 125
    */
    //% blockId=robotbit_stpcar_turn block="Car Turn|Degree %turn|Wheel Diameter(mm) %diameter|Track(mm) %track"
    //% group="Actuator" weight=87
    //% blockGap=50
    export function StpCarTurn(turn: number, diameter: number, track: number): void {
        if (!initialized) {
            initPCA9685()
        }
        let delay = 10240 * turn * track / 360 / diameter;
        setStepper(1, delay < 0);
        setStepper(2, delay > 0);
        delay = Math.abs(delay);
        basic.pause(delay);
        MotorStopAll()
    }
	
    //% blockId=robotbit_motor_run block="Motor|%index|speed %speed"
    //% group="Actuator" weight=85
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function MotorRun(index: Motors, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }
        speed = speed * 16; // map 255 to 4096
        if (speed >= 4096) {
            speed = 4095
        }
        if (speed <= -4096) {
            speed = -4095
        }
        if (index > 4 || index <= 0)
            return
        let pp = (index - 1) * 2
        let pn = (index - 1) * 2 + 1
        if (speed >= 0) {
            setPwm(pp, 0, speed)
            setPwm(pn, 0, 0)
        } else {
            setPwm(pp, 0, 0)
            setPwm(pn, 0, -speed)
        }
    }
	
	
   /**
     * Execute two motors at the same time
     * @param motor1 First Motor; eg: M1A, M1B
     * @param speed1 [-255-255] speed of motor; eg: 150, -150
     * @param motor2 Second Motor; eg: M2A, M2B
     * @param speed2 [-255-255] speed of motor; eg: 150, -150
    */
    //% blockId=robotbit_motor_dual block="Motor|%motor1|speed %speed1|%motor2|speed %speed2"
    //% group="Actuator" weight=84
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function MotorRunDual(motor1: Motors, speed1: number, motor2: Motors, speed2: number): void {
        MotorRun(motor1, speed1);
        MotorRun(motor2, speed2);
    }
	
    /**
     * Execute single motors with delay
     * @param index Motor Index; eg: M1A, M1B, M2A, M2B
     * @param speed [-255-255] speed of motor; eg: 150, -150
     * @param delay seconde delay to stop; eg: 1
    */
    //% blockId=robotbit_motor_rundelay block="Motor|%index|speed %speed|delay %delay|s"
    //% group="Actuator" weight=81
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function MotorRunDelay(index: Motors, speed: number, delay: number): void {
        MotorRun(index, speed);
        basic.pause(delay * 1000);
        MotorRun(index, 0);
    }
	
    //% blockId=robotbit_stop block="Motor Stop|%index|"
    //% group="Actuator" weight=80
    export function MotorStop(index: Motors): void {
        MotorRun(index, 0);
    }

    //% blockId=robotbit_stop_all block="Motor Stop All"
    //% group="Actuator" weight=79
    //% blockGap=50
    export function MotorStopAll(): void {
        if (!initialized) {
            initPCA9685()
        }
        for (let idx = 1; idx <= 4; idx++) {
            stopMotor(idx);
        }
    }
	
    //% blockId=powerbrick_ultrasonic block="Ultrasonic|Pin B %pin"
    //% group="Ultrasonic/Mic" weight=91
    export function Ultrasonic(pin:DigitalPin): number {
        // send pulse
        //let pin = PortDigi[port][0]
        pins.setPull(pin, PinPullMode.PullNone);
        pins.digitalWritePin(pin, 0);
        control.waitMicros(2);
        pins.digitalWritePin(pin, 1);
        control.waitMicros(10);
        pins.digitalWritePin(pin, 0);

        // read pulse
        let d = pins.pulseIn(pin, PulseValue.High, 25000);
        let ret = d;
        // filter timeout spikes
        if (ret == 0 && distanceBuf != 0) {
            ret = distanceBuf;
        }
        distanceBuf = d;
        return Math.floor(ret * 10 / 6 / 58);
    }

    //% blockId=powerbrick_sound block="Sound Sensor|Pin A %pin"
    //% weight=90
    //% group="Ultrasonic/Mic" blockGap=50
    export function SoundSensor(pin: AnalogPin): number {
        //let pin = PortAnalog[port]
        return pins.analogReadPin(pin)
    }

/*
    //% blockId=powerbrick_tracer_init block="Init Tracer |%pinA|%pinB"
    //% group="Linefollower" weight=10
    export function TracerInit(pinA: DigitalPin, pinB: DigitalPin): {
	tracerport[0] = pinA;
	tracerport[1] = pinB;
    }
*/

    //% blockId=powerbrick_tracer block="Tracer|pin %pin"
    //% group="Linefollower" weight=81
    export function Tracer(pin: DigitalPin): boolean {
        //let pin = PortDigi[port][slot]
    	//let pin=tracerport[i]
        pins.setPull(pin, PinPullMode.PullUp)
        return pins.digitalReadPin(pin) == 1
    }

    //% blockId=powerbrick_onTracerEvent block="on Tracer|%pin touch black"
    //% weight=80
    //% group="Linefollower" blockGap=50
    export function onTracerEvent(pin: DigitalPin, handler: () => void): void {
        //let pin = PortDigi[port][slot]
        pins.setPull(pin, PinPullMode.PullUp)
        pins.onPulsed(pin, PulseValue.High, handler)
    }

    //% blockId=powerbrick_bumper block="Bumper|pin %pin"
    //% group="Bumper" weight=71
    export function Bumper(pin: DigitalPin): boolean {
        //let pin = PortDigi[port][slot]
        pins.setPull(pin, PinPullMode.PullUp)
        return pins.digitalReadPin(pin) == 0
    }

    //% blockId=powerbrick_onBumperEvent block="on Bumper|%pin pressed"
    //% group="Bumper" weight=70
    export function onBumperEvent(pin: DigitalPin, handler: () => void): void {
        //let pin = PortDigi[port][slot]

        pins.setPull(pin, PinPullMode.PullUp)
        pins.onPulsed(pin, PulseValue.Low, handler)
    }


    //% blockId=powerbrick_dht11 block="DHT11|Pin B %pin|type %readtype"
    //% weight=60
    //% group="Environment" blockGap=50
    export function DHT11(pin: DigitalPin, readtype: DHT11Type): number {
        //let dht11pin = PortDigi[port][0]
	let dht11pin=pin
        pins.digitalWritePin(dht11pin, 0)
        basic.pause(18)
        let i = pins.digitalReadPin(dht11pin)
        pins.setPull(dht11pin, PinPullMode.PullUp);
        switch (readtype) {
            case 0:
                let dhtvalue1 = 0;
                let dhtcounter1 = 0;
                while (pins.digitalReadPin(dht11pin) == 1);
                while (pins.digitalReadPin(dht11pin) == 0);
                while (pins.digitalReadPin(dht11pin) == 1);
                for (let i = 0; i <= 32 - 1; i++) {
                    while (pins.digitalReadPin(dht11pin) == 0);
                    dhtcounter1 = 0
                    while (pins.digitalReadPin(dht11pin) == 1) {
                        dhtcounter1 += 1;
                    }
                    if (i > 15) {
                        if (dhtcounter1 > 2) {
                            dhtvalue1 = dhtvalue1 + (1 << (31 - i));
                        }
                    }
                }
                return ((dhtvalue1 & 0x0000ff00) >> 8);
                break;
            case 1:
                while (pins.digitalReadPin(dht11pin) == 1);
                while (pins.digitalReadPin(dht11pin) == 0);
                while (pins.digitalReadPin(dht11pin) == 1);
                let dhtvalue = 0;
                let dhtcounter = 0;
                for (let i = 0; i <= 32 - 1; i++) {
                    while (pins.digitalReadPin(dht11pin) == 0);
                    dhtcounter = 0
                    while (pins.digitalReadPin(dht11pin) == 1) {
                        dhtcounter += 1;
                    }
                    if (i > 15) {
                        if (dhtcounter > 2) {
                            dhtvalue = dhtvalue + (1 << (31 - i));
                        }
                    }
                }
                return Math.round((((dhtvalue & 0x0000ff00) >> 8) * 9 / 5) + 32);
                break;
            case 2:
                while (pins.digitalReadPin(dht11pin) == 1);
                while (pins.digitalReadPin(dht11pin) == 0);
                while (pins.digitalReadPin(dht11pin) == 1);

                let value = 0;
                let counter = 0;

                for (let i = 0; i <= 8 - 1; i++) {
                    while (pins.digitalReadPin(dht11pin) == 0);
                    counter = 0
                    while (pins.digitalReadPin(dht11pin) == 1) {
                        counter += 1;
                    }
                    if (counter > 3) {
                        value = value + (1 << (7 - i));
                    }
                }
                return value;
            default:
                return 0;
        }
    }

    //% blockId=powerbrick_soil block="Soil|Pin A %pin"
    //% weight=60
    //% group="Environment" blockGap=50
    export function Soil(pin: AnalogPin): number {
        //let pin = PortAnalog[port]
        return pins.analogReadPin(pin)
    }

    //% blockId=powerbrick_waterlevel block="Water level |Pin A %pin"
    //% weight=60
    //% group="Environment" blockGap=50
    export function WaterLevel(pin: AnalogPin): number {
        //let pin = PortAnalog[port]
        return pins.analogReadPin(pin)
    }

    //% blockId=powerbrick_infratemp block="Infra Temp  Pin A:SCL Pin B:SDA"
    //% weight=60
    //% group="InfraTemp"
    export function InfraTemp(): number {
        pins.i2cWriteNumber(27, 1, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(27, NumberFormat.Float32LE);
        return Math.roundWithPrecision(val, 2);
    }

    //% blockId=powerbrick_envtemp block="Environment Temp Pin A:SCL Pin B:SDA"
    //% weight=60
    //% group="InfraTemp"
    export function EnvironTemp(): number {
        pins.i2cWriteNumber(27, 2, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(27, NumberFormat.Float32LE);
        
        return Math.roundWithPrecision(val, 2);
    }

    //% blockId=powerbrick_led block="Infra LED R:%red G:%green Y:%yellow"
    //% weight=60
    //% group="InfraTemp" weight=50
    export function InfraLED(red: number, green: number, yellow: number): void {
        let buf = pins.createBuffer(5)
        buf[0] = 8
        buf[1] = red
        buf[2] = green
        buf[3] = yellow
        pins.i2cWriteBuffer(27, buf)
    }
    
    /**
     * turn on display
     */
    //% blockId="powerbrick_segment_on" block="turn on display Pin A:SCL Pin B:SDA"
    //% group="Led segment" weight=32
    export function segOn() {
        segCmd(_intensity * 16 + 1)
    }

    /**
     * turn off display
     */
    //% blockId="powerbrick_segment_off" block="turn off display"
    //% group="Led segment" weight=31
    export function segOff() {
        _intensity = 0
        segCmd(0)
    }

    /**
     * clear display content
     */
    //% blockId="powerbrick_segment_clear" block="clear display"
    //% group="Led segment" weight=33
    export function segClear() {
        segDat(0, 0)
        segDat(1, 0)
        segDat(2, 0)
        segDat(3, 0)
        dbuf = [0, 0, 0, 0]
    }

    /**
     * show a digital in given position
     * @param digit is number (0-15) will be shown, eg: 1
     * @param bit is position, eg: 0
     */
    //% blockId="powerbrick_segment_digit" block="show digit %num|at %bit"
    //% group="Led segment" weight=39
    //% num.max=9 num.min=0
    //% bit.max=3 bit.min=0
    export function segDigit(num: number, bit: number) {
        dbuf[bit % 4] = _SEG[num % 16]
        segDat(bit, _SEG[num % 16])
    }

    /**
     * show a number in display
     * @param num is number will be shown, eg: 100
     */
    //% blockId="powerbrick_segment_number" block="show number %num"
    //% group="Led segment" weight=38
    export function segShowNumber(num: number) {
        if (num < 0) {
            segDat(0, 0x40) // '-'
            num = -num
        }
        else
            segDigit(Math.idiv(num, 1000) % 10, 0)
        segDigit(num % 10, 3)
        segDigit(Math.idiv(num, 10) % 10, 2)
        segDigit(Math.idiv(num, 100) % 10, 1)
    }


    /**
     * show Dot Point in given position
     * @param bit is positiion, eg: 0
     * @param show is true/false, eg: true
     */
    //% blockId="powerbrick_segment_dot" block="show dot point %bit|Show %show"
    //% group="Led segment" weight=35
    //% bit.max=3 bit.min=0
    export function segShowDpAt(bit: number, show: boolean) {
        if (show) segDat(bit, dbuf[bit % 4] | 0x80)
        else segDat(bit, dbuf[bit % 4] & 0x7F)
    }

    /**
     * set display intensity
     * @param dat is intensity of the display, eg: 3
     */
    //% blockId="powerbrick_segment_intensity" block="set intensity %dat"
    //% group="Led segment" weight=34
    //% dat.max=8 dat.min=0
    export function segSetIntensity(dat: number) {
        if ((dat < 0) || (dat > 8))
            return;
        if (dat == 0)
            segOff()
        else {
            _intensity = dat
            segCmd((dat << 4) | 0x01)
        }
    }


    function calcSum(buf: Buffer, start: number, end: number): number {
        let sum = 0;
        for (let i = start; i <= end; i++) {
            sum += buf[i];
        }
        return sum;
    }

    //% blockId=powerbrick_mp3_connect block="MP3 Connect|Pin A TX %txpin| Pin B RX %rxpin"
    //% group="MP3" weight=39
    export function MP3Connect(txpin: SerialPin, rxpin: SerialPin): void {
        //let pin0 = PortSerial[port][0]
        //let pin1 = PortSerial[port][1]
        // todo: fiber may freeze on steam reading
        serial.redirect(txpin, SerialPin.P16, BaudRate.BaudRate9600)
    }

    //% blockId=powerbrick_mp3_play block="MP3 Play|%PrevNext"
    //% group="MP3" weight=38
    export function MP3Play(pn: PrevNext): void {
        let buf = pins.createBuffer(5);
        buf[0] = 0x7e;
        buf[1] = 0x03;
        buf[2] = pn;
        buf[3] = buf[1] + buf[2];
        buf[4] = 0xef;
        serial.writeBuffer(buf)
    }

    //% blockId=powerbrick_mp3_volumn block="MP3 Volume|%volumn"
    //% volumn.min=0 volumn.max=31
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% group="MP3" weight=37
    export function MP3Volumn(volumn: number): void {
        let buf = pins.createBuffer(6);
        buf[0] = 0x7e;
        buf[1] = 0x04;
        buf[2] = 0xae;
        buf[3] = volumn;
        buf[4] = calcSum(buf, 1, 3);
        buf[5] = 0xef;
        serial.writeBuffer(buf)
    }

    //% blockId=powerbrick_mp3_playindex block="MP3 Play Index|%index"
    //% group="MP3" weight=37
    export function MP3PlayIndex(index: number): void {
        let buf = pins.createBuffer(7);
        if (index == 0) {
            index = 1;
        }
        buf[0] = 0x7e;
        buf[1] = 0x05;
        buf[2] = 0xa2;
        buf[3] = 0;
        buf[4] = index;
        buf[5] = calcSum(buf, 1, 4);
        buf[6] = 0xef;
        serial.writeBuffer(buf)
    }

    //% blockId=powerbrick_mp3_playname block="MP3 Play Name|%name"
    //% weight=36
    //% group="MP3" blockGap=50
    export function MP3PlayName(str: string): void {
        let len = str.length;
        if (len > 8) len = 8;
        let buf = pins.createBuffer(len + 5);
        buf[0] = 0x7e;
        buf[1] = len + 3;
        buf[2] = 0xa3;
        for (let i = 0; i < len; i++) {
            buf[3 + i] = str.charCodeAt(i);
        }
        buf[len + 3] = calcSum(buf, 1, len + 2);
        buf[len + 4] = 0xef;
        serial.writeBuffer(buf)
    }

    //% blockId=powerbrick_gc_mode block="Gesture/Color Pin A:SCL Pin B:SDA mode|%mode"
    //% group="Color/Gesture" weight=29
    export function GC_MODE(mode: GCMode): void {
        i2cwrite(KC_ADDR, KC_MODE, mode);
    }

    //% blockId=powerbrick_gc_color block="Gesture/Color Color Hue"
    //% group="Color/Gesture" weight=28
    export function GC_Color(): number {
        pins.i2cWriteNumber(KC_ADDR, KC_READCOLOR, NumberFormat.UInt8BE);
        let buff = pins.i2cReadBuffer(KC_ADDR, 2);
        return buff[0] * 2;
    }

    //% blockId=powerbrick_gc_brightness block="Gesture/Color Brightness"
    //% group="Color/Gesture" weight=27
    export function GC_Brightness(): number {
        pins.i2cWriteNumber(KC_ADDR, KC_READCOLOR, NumberFormat.UInt8BE);
        let buff = pins.i2cReadBuffer(KC_ADDR, 2);
        return buff[1];
    }

    //% blockId=powerbrick_gc_ledpwm block="Gesture/Color LED Brightness|%pwm"
    //% pwm.min=0 pwm.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% group="Color/Gesture" weight=26
    export function GC_LEDPWM(pwm: number): void {
        i2cwrite(KC_ADDR, KC_LEDPWM, pwm);
    }

    //% blockId=powerbrick_gc_ledonoff block="Gesture/Color LED|%index|On/Off %onoff"
    //% group="Color/Gesture" weight=25
    export function GC_LEDONOFF(index: GCLed, onoff: GCOnOff): void {
        let buf = pins.createBuffer(3)
        buf[0] = KC_LEDONOFF
        buf[1] = index
        buf[2] = onoff
        pins.i2cWriteBuffer(KC_ADDR, buf)
        basic.pause(1)
    }

    //% blockId=powerbrick_gc_ledbit block="Gesture/Color|LED1 %l1|LED2 %l2|LED3 %l3|LED4 %l4"
    //% group="Color/Gesture" weight=24
    export function GC_LEDBIT(l1: GCOnOff, l2: GCOnOff, l3: GCOnOff, l4: GCOnOff): void {
        let buf = pins.createBuffer(2)
        buf[0] = KC_LEDBIT
        buf[1] = l1 * 1 + l2 * 2 + l3 * 4 + l4 * 8;
        pins.i2cWriteBuffer(KC_ADDR, buf)
        basic.pause(1)
    }

    //% blockId=powerbrick_gc_proximity block="Gesture/Color Proximity"
    //% group="Color/Gesture" weight=23
    export function GC_PROXIMITY(): number {
        return i2cread(KC_ADDR, KC_PROXIMITY)
    }

    //% blockId=powerbrick_gc_gesture block="Gesture/Color last gesture"
    //% group="Color/Gesture" weight=22
    export function GC_Gesture(): number {
        return i2cread(KC_ADDR, KC_GESTURE)
    }

    //% blockId=powerbrick_gc_rgb block="Gesture/Color RGB|%rgb"
    //% group="Color/Gesture" weight=21
    //% blockGap = 50
    export function GC_RGB(rgb: GCRgb): number {
        pins.i2cWriteNumber(KC_ADDR, KC_READCOLORRAW, NumberFormat.UInt8BE);
        let buff = pins.i2cReadBuffer(KC_ADDR, 4);
        return buff[rgb];
    }

    //% blockId=powerbrick_rfidprobe block="RFID Probe Pin A:SCL Pin B:SDA"
    //% weight=21
    //% group="RFID" 
    export function RfidProbe(): void {
        let stat = i2cread(RFID_ADDR, RFID_STATUS);
        if (stat == 1) {
            if (onRfidPresent) {
                onRfidPresent();
                RfidStop();
            }
        }
    }

    //% blockId=powerbrick_onrfidpresent block="RFID Present and wait"
    //% weight=20
    //% group="RFID" 
    export function RfidPresent(handler: () => void): void {
        onRfidPresent = handler;
    }

    //% blockId=powerbrick_rfiduuid block="RFID UUID"
    //% weight=19
    //% group="RFID"
    export function RfidUUID(): string {
        pins.i2cWriteNumber(RFID_ADDR, RFID_UUID, NumberFormat.UInt8BE);
        let uuid = pins.i2cReadBuffer(RFID_ADDR, 4)
        let uuidReverse = pins.createBuffer(4)
        // reverse byte order to micropython type~
        uuidReverse[0] = uuid[3]
        uuidReverse[1] = uuid[2]
        uuidReverse[2] = uuid[1]
        uuidReverse[3] = uuid[0]
        return uuidReverse.toHex();
    }

    //% blockId=powerbrick_rfidwrite block="RFID Write sector|%sector block|%block text|%txt"
    //% weight=18
    //% group="RFID" 
    export function RfidWrite(sector: RfidSector, block: RfidBlock, txt: string): void {
        let buf = pins.createBuffer(19)
        buf[0] = RFID_WRITE
        buf[1] = sector
        buf[2] = block
        let len = txt.length
        if (len > 16) len = 16
        for (let i = 0; i < len; i++) {
            buf[3 + i] = txt.charCodeAt(i)
        }
        pins.i2cWriteBuffer(RFID_ADDR, buf)
        basic.pause(100)
    }

    //% blockId=powerbrick_rfidread block="RFID Read sector|%sector block|%block"
    //% weight=17
    //% group="RFID" 
    export function RfidRead(sector: RfidSector, block: RfidBlock): string {
        let retry: number = 5;
        let buf = pins.createBuffer(3)
        buf[0] = RFID_READCMD
        buf[1] = sector
        buf[2] = block
        pins.i2cWriteBuffer(RFID_ADDR, buf)

        while (retry) {
            basic.pause(100);
            let stat = i2cread(RFID_ADDR, RFID_STATUS);
            if (stat == RfidStat.READ_SUCC) {
                let ret = '';
                pins.i2cWriteNumber(RFID_ADDR, RFID_READOUT, NumberFormat.UInt8BE);
                let rxbuf = pins.i2cReadBuffer(RFID_ADDR, 16)
                for (let i = 0; i < 16; i++) {
                    if (rxbuf[i] >= 0x20 && rxbuf[i] < 0x7f) {
                        ret += String.fromCharCode(rxbuf[i]) // valid ascii
                    }
                }
                return ret;
            }
            retry--;
        }
        return '';
    }


    function RfidStop(): void {
        let buf = pins.createBuffer(1)
        buf[0] = RFID_STOP
        pins.i2cWriteBuffer(RFID_ADDR, buf)
    }


	
    //% blockId="powerbrick_rgbattach" block="RGB connect at Pin B %pin"
    //% weight=10 blockGap=8
    //% group="RGB"
    export function rgbConnect(pin: DigitalPin) {
        //rgbPin = PortDigi[port][0];
	rgbPin=pin
        basic.pause(20)
        showColor(colors(NeoPixelColors.Black))
    }

    /**
     * Shows a rainbow pattern on all LEDs. 
     * @param startHue the start hue value for the rainbow, eg: 1
     * @param endHue the end hue value for the rainbow, eg: 360
     **/
	

    //% blockId="powerbrick_rgbrainbow" block="show rainbow from %startHue|to %endHue"
    //% weight=85 blockGap=8
    //% group="RGB" 
    export function showRainbow(startHue: number = 1, endHue: number = 360) {
        const saturation = 100;
        const luminance = 50;
        const steps = RGB_PIX;

        //hue
        const h1 = startHue;
        const h2 = endHue;
        const hDistCW = ((h2 + 360) - h1) % 360;
        const hStepCW = Math.idiv((hDistCW * 100), steps);
        const hDistCCW = ((h1 + 360) - h2) % 360;
        const hStepCCW = Math.idiv(-(hDistCCW * 100), steps);
        let hStep: number;
        hStep = hStepCW;

        const h1_100 = h1 * 100; //we multiply by 100 so we keep more accurate results while doing interpolation
        //sat
        const s1 = saturation;
        const s2 = saturation;
        const sDist = s2 - s1;
        const sStep = Math.idiv(sDist, steps);
        const s1_100 = s1 * 100;

        //lum
        const l1 = luminance;
        const l2 = luminance;
        const lDist = l2 - l1;
        const lStep = Math.idiv(lDist, steps);
        const l1_100 = l1 * 100

        //interpolate

        setPixelRGB(0, hsl(startHue, saturation, luminance));
        for (let i = 1; i < steps - 1; i++) {
            const h = Math.idiv((h1_100 + i * hStep), 100) + 360;
            const s = Math.idiv((s1_100 + i * sStep), 100);
            const l = Math.idiv((l1_100 + i * lStep), 100);
            setPixelRGB(i, hsl(h, s, l));
        }
        setPixelRGB(steps - 1, hsl(endHue, saturation, luminance));
        rgbShow();
    }

    //% blockId="neopixel_set_strip_color" block="show color %rgb=neopixel_colors"
    //% weight=85 blockGap=8
    //% group="RGB" 
    export function showColor(rgb: number) {
        let red = unpackR(rgb);
        let green = unpackG(rgb);
        let blue = unpackB(rgb);

        const br = rgbBright;
        if (br < 255) {
            red = (red * br) >> 8;
            green = (green * br) >> 8;
            blue = (blue * br) >> 8;
        }
        for (let i = 0; i < 64; ++i) {
            setBufferRGB(i * 3, red, green, blue)
        }

        rgbShow();
    }

    /**
     * Converts a hue saturation luminosity value into a RGB color
     * @param h hue from 0 to 360
     * @param s saturation from 0 to 99
     * @param l luminosity from 0 to 99
     **/

    //% blockId=neopixelHSL block="hue %h|saturation %s|luminosity %l"
    //% group="RGB" 
    export function hsl(h: number, s: number, l: number): number {
        h = Math.round(h);
        s = Math.round(s);
        l = Math.round(l);

        h = h % 360;
        s = Math.clamp(0, 99, s);
        l = Math.clamp(0, 99, l);
        let c = Math.idiv((((100 - Math.abs(2 * l - 100)) * s) << 8), 10000); //chroma, [0,255]
        let h1 = Math.idiv(h, 60);//[0,6]
        let h2 = Math.idiv((h - h1 * 60) * 256, 60);//[0,255]
        let temp = Math.abs((((h1 % 2) << 8) + h2) - 256);
        let x = (c * (256 - (temp))) >> 8;//[0,255], second largest component of this color
        let r$: number;
        let g$: number;
        let b$: number;
        if (h1 == 0) {
            r$ = c; g$ = x; b$ = 0;
        } else if (h1 == 1) {
            r$ = x; g$ = c; b$ = 0;
        } else if (h1 == 2) {
            r$ = 0; g$ = c; b$ = x;
        } else if (h1 == 3) {
            r$ = 0; g$ = x; b$ = c;
        } else if (h1 == 4) {
            r$ = x; g$ = 0; b$ = c;
        } else if (h1 == 5) {
            r$ = c; g$ = 0; b$ = x;
        }
        let m = Math.idiv((Math.idiv((l * 2 << 8), 100) - c), 2);
        let r = r$ + m;
        let g = g$ + m;
        let b = b$ + m;
        return packRGB(r, g, b);
    }

	

    //% blockId="neopixel_clear" block="RGB clear"
    //% group="RGB" 
    export function rgbClear(): void {
        rgbBuf.fill(0, 0, RGB_PIX * 3);
        rgbShow();
    }

	

    //% blockId=setRGBPix block="RGB PIX%pix Color%rgb=neopixel_colors"
    //% group="RGB" 
    export function setRGBPix(pix: number, rgb: number): void {
        if (pix < 0 || pix >= RGB_PIX)
            return;
        pix = pixMap(pix);
        pix = (pix) * 3;

        let red = unpackR(rgb);
        let green = unpackG(rgb);
        let blue = unpackB(rgb);

        let br = rgbBright;
        if (br < 255) {
            red = (red * br) >> 8;
            green = (green * br) >> 8;
            blue = (blue * br) >> 8;
        }
        setBufferRGB(pix, red, green, blue)
    }

	

    //% blockId=setRGBXy block="RGB X%x Y%y Color%rgb=neopixel_colors"
    //% group="RGB" 
    export function setRGBXy(x: number, y: number, rgb: number): void {
        setRGBPix(x + y * RGB_M, rgb)
    }

	

    //% blockId=showNum block="Show number %num Color%rgb=neopixel_colors"
    //% group="RGB" blockGap=16
    export function showNum(num: number, rgb: number): void {
        num = num % 100;
        let n1 = Math.floor(num / 10);
        let n0 = num % 10;
        let pix = 0;
        rgbBuf.fill(0, 0, RGB_PIX * 3);
        if (n1 > 0) {
            pix = FontNum[n1];
            for (let x = 0; x < 3; x++) {
                let p = (pix >> x * 8) & 0xff;
                for (let y = 0; y < 8; y++) {
                    if ((p >> y) & 1) {
                        setRGBXy(x + 1, y, rgb)
                    }
                }
            }
        }

        pix = FontNum[n0];
        for (let x = 0; x < 3; x++) {
            let p = (pix >> x * 8) & 0xff;
            for (let y = 0; y < 8; y++) {
                if ((p >> y) & 1) {
                    setRGBXy(x + 5, y, rgb)
                }
            }
        }
        rgbShow()
    }
	

    //% blockId=neopixel_colors block="%color"
    //% group="RGB" 
    export function colors(color: NeoPixelColors): number {
        return color;
    }

    //% blockId=rgbColor block="Color red %red|green %green|blue %blue"
    //% group="RGB" 
    export function rgb(red: number, green: number, blue: number): number {
        return packRGB(red, green, blue);
    }

    //% blockId=rgbBrightness block="RGB brightness %brightness"
    //% group="RGB" blockGap=16
    export function setBrightness(brightness: number): void {
        rgbBright = brightness & 0xff
    }

    //% blockId=rgbPixRotate block="rotate pixels by %offset"
    //% group="RGB" 
    export function rgbPixRotate(offset: number = 1): void {
        rgbBuf.rotate(-offset * 3, 0, 64 * 3)
    }

    //% blockId=rgbShow block="RGB show"
    //% group="RGB" 
    export function rgbShow() {
        ws2812b.sendBuffer(rgbBuf, rgbPin);
    }

    /*
    export enum RGBIconName {
        //% block="riven"
        //% blockImage=1
        //% iconURL="https://kittenbot.cn/bbs/uc_server/avatar.php?uid=2&size=middle"
        riven = 0,
    }
    
    //% blockId=rgb_show_icon
    //% block="show icon %i"
    //% group="RGB" 
    //% i.fieldEditor="gridpicker"
    //% i.fieldOptions.width="400" i.fieldOptions.columns="5"
    //% i.fieldOptions.itemColour="black" i.fieldOptions.tooltips="true"
    export function showIcon(icon: RGBIconName, interval = 600) {
    }
    */

    function pinMap(x: number, y: number): number {
        let t = y;
        y = x; x = t;
        if (y % 2 == 1) { // s-type connection on hardware
            x = RGB_M - x - 1;
        }
        let pix = y * RGB_M + x;
        return pix
    }

    function pixMap(pix: number): number {
        let x = pix % RGB_M;
        let y = Math.floor(pix / RGB_M);
        return pinMap(x, y)
    }


    function setPixelRGB(pixeloffset: number, rgb: number): void {
        if (pixeloffset < 0 || pixeloffset >= RGB_PIX)
            return;

        let stride = 3;
        pixeloffset = (pixeloffset) * stride;

        let red = unpackR(rgb);
        let green = unpackG(rgb);
        let blue = unpackB(rgb);

        let br = rgbBright;
        if (br < 255) {
            red = (red * br) >> 8;
            green = (green * br) >> 8;
            blue = (blue * br) >> 8;
        }
        setBufferRGB(pixeloffset, red, green, blue)
    }


    function packRGB(a: number, b: number, c: number): number {
        return ((a & 0xFF) << 16) | ((b & 0xFF) << 8) | (c & 0xFF);
    }

    function unpackR(rgb: number): number {
        let r = (rgb >> 16) & 0xFF;
        return r;
    }
    function unpackG(rgb: number): number {
        let g = (rgb >> 8) & 0xFF;
        return g;
    }
    function unpackB(rgb: number): number {
        let b = (rgb) & 0xFF;
        return b;
    }


}
