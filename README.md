# PowerEDU

PowerEDU is an extension that adds support for Powerbrick series modules to Robotbit.

## Feature

* This extension is a combination of KittenBot's Robotbit Extension(https://github.com/KittenBot/pxt-robotbit) and PowerBrick Extension(https://github.com/KittenBot/pxt-powerbrick). These extensions were previously incompatible with each other, PowerEDU solved the incompatibility thus allowing Robotbit users to program and use modules from Powerbrick series.

1. Modules previously only exclusive to Powerbrick users are made available for Robotbit users.

2. Combining the powerful capabilities of Robotbit and the wide range of sensors of Powerbrick.

3. Powerbrick series modules and Sugar series modules can be used together.


## Basic usage
* The Ultrasonic & Sound sensor module works and feeds back the data of distance and sound to the micro:bit

```blocks

    input.onButtonPressed(Button.A, function () {
        basic.showNumber(PowerEDU.Ultrasonic(DigitalPin.P12))
    })
    input.onButtonPressed(Button.B, function () {
        basic.showNumber(PowerEDU.SoundSensor(AnalogPin.P2))
    })

```

---

* The buzzer sounds when the line tracker module detects a black line

```blocks

    basic.forever(function () {
        if (PowerEDU.Tracer(DigitalPin.P2) || PowerEDU.Tracer(DigitalPin.P12)) {
            music.playTone(262, music.beat(BeatFraction.Whole))
        }
    })

```

---

* Press the Bumpers and the buzzer will sound

```blocks

    basic.forever(function () {
        if (PowerEDU.Bumper(DigitalPin.P2) || PowerEDU.Bumper(DigitalPin.P12)) {
            music.playTone(262, music.beat(BeatFraction.Whole))
        }
    })

```

---

* Temperature and humidity data will be displayed on micro:bit

```blocks

    input.onButtonPressed(Button.A, function () {
        basic.showNumber(PowerEDU.DHT11(DigitalPin.P1, PowerEDU.DHT11Type.TemperatureC))
        basic.showNumber(PowerEDU.DHT11(DigitalPin.P1, PowerEDU.DHT11Type.Humidity))
    })

```

---

* Let the servo(grey / limited in -45-225) and the motor(red / limited in -255-255) work.

```blocks

    basic.forever(function () {
        PowerEDU.MotorRun(PowerEDU.Motors.M1A, -255)
        PowerEDU.GeekServo(PowerEDU.Servos.S1, -45)
        basic.pause(2000)
        PowerEDU.GeekServo(PowerEDU.Servos.S1, 225)
        PowerEDU.MotorRun(PowerEDU.Motors.M1A, 255)
        basic.pause(2000)
    })
    
```

---

* The color & gesture module displays the color(Hue) value on the micro:bit

```blocks

    input.onButtonPressed(Button.A, function () {
        basic.showNumber(PowerEDU.GC_Color())
    })
    PowerEDU.GC_MODE(PowerEDU.GCMode.ColorSensor)

```

---

* The color & gesture module will send 1~4 to the micro:bit means 4 directions, and 0 means no gestrue

```blocks

    let now_ges = 0
    PowerEDU.GC_MODE(PowerEDU.GCMode.Gesture)
    basic.forever(function () {
        now_ges = PowerEDU.GC_Gesture()
        if (!(now_ges == 0)) {
            basic.showNumber(now_ges)
        }
    })

```

---

* The RFID module detects the RFID card, it will displays the UUID of the card and writes a piece of information to the card

```blocks

    PowerEDU.RfidPresent(function () {
        basic.showString(PowerEDU.RfidUUID())
        PowerEDU.RfidWrite(PowerEDU.RfidSector.S1, PowerEDU.RfidBlock.B0, "hello")
        basic.showString(PowerEDU.RfidRead(PowerEDU.RfidSector.S1, PowerEDU.RfidBlock.B0))
    })
    basic.forever(function () {
        PowerEDU.RfidProbe()
    })

```

---

* Let the RGB module display

```blocks

    input.onButtonPressed(Button.A, function () {
        PowerEDU.showColor(PowerEDU.colors(PowerEDU.NeoPixelColors.Red))
        PowerEDU.rgbShow()
    })
    PowerEDU.rgbConnect(DigitalPin.P1)

```

---

* MP3 will play mp3 audio stored in the tf card

```blocks

    input.onButtonPressed(Button.A, function () {
        PowerEDU.MP3Play(PowerEDU.PrevNext.Play)
    })
    PowerEDU.MP3Connect(SerialPin.P12, SerialPin.P2)

```


## License

MIT

## Supported targets

* for PXT/microbit
* for PXT/meowbit