# Sourcing notes

The table above links one listing per part plus a cheaper or more trustworthy alternative. Listings come and go, so here is what to look for when the linked one is gone or looks wrong. Prices change; check before you order.

Many of these parts are common in a robotics parts drawer. They are all listed so that you can build a Tailwhip from this page; if you already have them, only the items marked BUY are new.

## Boards: ESP32, IMU, motor driver, camera

- **ESP32:** classic ESP32-WROOM-32, not C3, S2 or S3. You need two: one on the robot, one plugged into the laptop as the USB radio bridge. 30-pin and 38-pin DevKits both fit; the robot board sits on the spine with its header pins pointing back. Adafruit's HUZZAH32 Feather ($19.95) is genuine but a different shape.
- **IMU:** GY-521 (MPU-6050). Cheap 5-packs are fine. Adafruit's MPU-6050 ($12.95) is a genuine sensor on a different board, so the mount would need changing.
- **Motor driver:** MX1508 module, 2 to 10 V, two inputs per motor and no enable pins. The chip's datasheet rates one channel at 1.5 A continuous and the other at only 0.8 A continuous, and modules rarely say which is which. A TT motor can pull 1.1 to 1.5 A when stalled, so do not hold the robot against a wall with the motors on.
- **ESP32-CAM:** the AI-Thinker style board with the OV2640 camera. Only needed at stage 4. A kit with the small USB programmer board (ESP32-CAM-MB) saves wiring up a separate USB-serial adapter.

## Motors, wheels and servos

- **TT motor:** the common yellow 1:48 gearmotor with a straight (not 90-degree) shaft, 3 to 6 V. Most listings bundle a 65 mm wheel with each motor. Adafruit sells the motor at $2.95 and 65 mm wheels at $1.50.
- **Wheels:** 65 mm with a rubber tyre. Hard plastic tyres slip on smooth floors.
- **SG-90 servos:** must say 180 degrees. A 360-degree or "continuous rotation" SG-90 looks identical and will not hold a position.
- **MG90S servos (buy two):** the same body as the SG-90 with metal gears, so one drops straight into the tail mount if the tail SG-90 strips its gears. Again, the listing must say 180 degrees. Waveshare sells them at $2.59 each; Adafruit's MG90D ($9.95) is the trustworthy option.

## Sensors and regulators

- **IR sensor:** the TCRT5000 module with an LM393 comparator and a blue trim pot, with a digital "DO" output. Power it from 3.3 V so its output is safe for the ESP32.
- **Mini360 buck:** the tiny adjustable board, 4.75 to 23 V in. Most listings rate it 1.8 A continuous and 3 A surge. It comes set to a random voltage: **set it to 5.0 V with a meter before you connect anything.**

## Battery cells (18650)

- **Buy cells from a battery specialist, not Amazon or eBay.** Cell listings on the big marketplaces are often counterfeit or rewrapped cheap cells. Good shops: 18650batterystore.com, imrbatteries.com, liionwholesale.com, Illumn.
- **Never buy "UltraFire", "GTF", or anything claiming 9900 mAh.** No real 18650 holds more than about 3,500 mAh.
- **Pick a genuine high-drain cell.** Good choices, in stock at 18650batterystore.com when checked: Molicel P28A ($5.99, 35 A continuous), Samsung 25R ($4.99, 20 A), Samsung 30Q ($6.99, 15 A) and Sony/Murata VTC6 ($7.99, 15 A). The P28A is also $5.99 at imrbatteries.com.
- **Why high-drain:** these cells have low internal resistance, so the voltage barely sags when the robot pulls 1 to 5 A. The ESP32 is less likely to brown out when the motors kick. A real 15 to 20 A continuous rating also leaves a big margin over the robot's few amps.
- **Flat top, unprotected.** The BMS (item 13) protects the pack, so you do not need protected cells, which are also longer and may not fit. Most spring holders take flat tops; buy button tops only if your holder needs them.
- **Buy a matched pair:** same model, same shop, bought together. Charge both full before you build the pack.

## Battery holder

- 2 x 18650 in series, two leads out, about 41 x 77.5 x 20.5 mm outside (what the CAD assumes). SparkFun's PRT-12900 ($1.95) is 74.5 x 40.8 x 17.7 mm but does not say series or parallel: with cells in, the leads should read 7 to 8.4 V.
- The BMS also needs the middle of the pack (between the two cells). Solder a thin wire to the strap that joins the cells, or pick a holder where that strap is reachable.

## Battery protection board (BMS)

- **2S, 7.4 V nominal / 8.4 V full, at least 5 A continuous.** The robot pulls up to about 4.5 A for a second.
- **Balancing or not.** A balanced board bleeds a little current from the higher cell near full charge, so the two cells stay matched over months of charging through the robot. That is why the table picks one. An unbalanced board is smaller but lets the cells drift; if you use one, check each cell with a meter or a smart charger every few weeks.
- **The listed board:** HX-2S-JH20 from ShillehTek, $6.99. 10 A continuous, about 15 A for short bursts; over-charge cutoff 4.25 to 4.35 V per cell, over-discharge cutoff 2.4 to 2.6 V per cell; passive balancing at about 40 mA. It is about 50 x 22 x 3.5 mm, **larger than the 30 x 16 mm slot in the CAD**, so the slot has to grow.
- Cheap unbalanced "2S 5A" boards are typically 5 A working, 7 A trip, over-discharge 2.5 to 3.0 V. Check the over-current figure: anything that trips below about 6 A will cut out when both motors start at once.
- After wiring, a BMS may stay "locked" with no output until a charger is plugged in once.

## Fuse and power switch

- **Polyfuse:** radial PTC, 3 A hold, 6 A trip, 16 V or more: Littelfuse RUEF300 ($0.62 at DigiKey) or Bourns MF-R300.
- **The power switch carries all the motor current:** 3 to 4 A peaks and 1 to 1.5 A on average. **Tiny SS12D00-style slide switches are rated only about 0.3 to 0.5 A and will burn out. Do not use one.** Also beware: in some slide-switch part numbers "3A" is the actuator length (3 mm), not the current rating. Read the rating in the spec table.
- **First choice:** Pololu Mini MOSFET Slide Switch LV, item 2810 ($4.49). A tiny slide switch turns a MOSFET on and off, so the switch itself carries no current. Good for 1.8 to 16 V, 3 A continuous with only a 55 °C rise, and it protects against a reversed battery. The board is 15.2 x 15.2 mm; the lever sticks out about 1 mm.
- **Second choice:** KCD11 mini rocker, rated 3 A 250 VAC / 6 A 125 VAC. It snaps into a 10 x 15 mm hole and is about 18 mm deep behind the panel.
- Either one is bigger than the 6.2 x 10 x 6 mm slide switch the CAD currently assumes.

## Charging

- **Charger:** a 2S Li-ion charger, 8.4 V, 1 A, CC/CV, with a 5.5 x 2.1 mm plug. Not 7.4 V, not 9 V, and not a 12 V adapter. Check the plug size (some are 5.5 x 2.5 mm) and that the centre is positive. A red/green LED that turns green when full is handy.
- **Charge jack:** a 5.5 x 2.1 mm female jack with short wires, connected to the BMS P+ and P-. Check which wire is centre positive with a meter before soldering.
- **Smart charger (optional):** an XTAR VC2S ($14.99 at imrbatteries.com) or Nitecore i2 charges cells one at a time and shows each cell's voltage, which is the easy way to check that the two cells still match.

## Small electronics

- **Capacitor:** 470 µF, 16 V (or 10 V), low ESR, across the servo 5 V rail. Mind the polarity stripe.
- **Resistors:** 100 kΩ and 33 kΩ for the battery divider, plus two more 100 kΩ as IR pull-downs. Check that any kit lists 33 kΩ; the Aniann 1280-piece kit does, smaller kits may not.
- **100 nF ceramic capacitor** (marked 104) across the divider.
- **JST-XH 2-pin pairs** on the motor leads, so the motors unplug. Pre-wired pairs (Adafruit, $0.95 a pair) save buying a crimp tool. JST-XH is really 2.5 mm pitch.
- **Header pins and perfboard:** 2.54 mm male header for the two servo plugs on a small perfboard. A 4 x 6 cm board cut down works.
- **Dupont jumpers:** female-female, 10 cm, pushed onto the ESP32 pins.

## Screws, wire, ties and filament

- **M3 x 70 mm** socket head, two, clamping both motors through the core. Partly threaded is fine. M3 threaded rod cut to length works too.
- **M3 assortment kit** with M3 x 8, M3 x 10 and nuts: one kit covers the rest of the screws.
- **Tail pivot:** an M3 shoulder screw with a 4 mm shoulder, 8 mm long (search "shoulder bolt" or "ISO 7379 M3"; McMaster, Misumi and Zoro all stock it). Fallback: an M3 x 16 screw through an 8 mm long, 4 mm brass spacer.
- **Wire:** silicone 22 AWG red and black for battery, BMS, switch, motor and buck wiring (about 0.6 m); silicone 26 AWG in a few colours for signals (about 1.5 m).
- **Zip ties** 2.5 x 100 mm, and a heat-shrink assortment.
- **Filament:** about 92 g of PLA for the body and 10 g of ABS for the tail, which has to flex.
