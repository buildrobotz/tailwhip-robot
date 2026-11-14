Every bought part for one Tailwhip is in the tables below, including the parts many builders already have (marked "Often already in a robotics parts drawer"), so that you can build one from this page. The list is AI-generated and unbuilt: check each listing before you order.

## What it costs

The table is split into three groups, because a single total mixes up what the robot needs with what is optional
and what is a workshop tool.

| Group | Per robot | What is in it |
|---|---|---|
| **Core robot** | **about $62** | Everything the robot needs to rear up and balance: two ESP32s (robot and radio bridge), MPU6050, MX1508, two TT motors and wheels, two SG-90s, two IR modules, two bucks, two 18650 cells, holder, BMS, fuse, switch, charge jack, wiring, the screws it uses and filament |
| Core, if you have the usual parts drawer | about $26 | The same, without the ESP32s, MPU6050, MX1508, TT motors, wheels, SG-90s, IR modules and bucks |
| Optional | about $22 | ESP32-CAM (milestone 6, the camera head) and two MG90S (only if the tail SG-90 strips its gears) |
| Shop tools | about $30 | A 2S 8.4 V charger, an M3 screw kit and a heat-shrink kit: bought once, used for many robots |

"Per robot" uses the per-unit price inside each pack. Most parts come in packs of 2 to 10, so buying every pack new
for a single robot costs far more up front (about $250 for the core packs) but leaves the parts for several more
robots: it is the right way to think about a class or a club, not one build.

**Compared with Ouro ($87):** the robot itself is cheaper, mostly because two $3 TT gearmotors and two $2.50
SG-90s replace eight MG90S servos. What both robots share sets the floor: two ESP32s (about $14), a battery
and its charger.

Prices change. Check each listing before you order.

## Where it departs from the SPEC's parts list

| Part | Change | Why |
|---|---|---|
| Power switch | Rated 3 A or more (Pololu 2810 MOSFET slide switch, or a KCD11 mini rocker), not a tiny slide switch | It carries all the motor current. SS12D00-type switches are rated 0.3-0.5 A [D]. Both rated choices are bigger than the CAD's 6.2 x 10 x 6 mm switch pocket: the Pololu board is 15.2 x 15.2 mm [D], the KCD11 face 10 x 15 mm and about 18 mm deep [D]. |
| BMS | One with balancing (HX-2S-JH20 type, 10 A) | Unbalanced boards let the two cells drift apart over many charges, and the robot charges in place. It is about 50 x 22 x 3.5 mm [D], bigger than the CAD's 30 x 16 mm slot. |
| Battery holder | Needs a wire soldered to the strap between the cells | The BMS needs the middle-of-pack (BM) connection; a two-lead holder does not bring it out. |
| Charger and charge jack | Added | The cells charge in place through the BMS. |
| Junction board, resistors, 100 nF | Added | The battery divider and the IR pull-downs, and a place for the servo capacitor and servo plugs. |

## The 18650 cells

Use genuine high-drain cells from a battery specialist, as a matched pair (same model, bought together). A Molicel P28A (2800 mAh, 35 A continuous, $5.99 [D]) is the first choice; Samsung 25R, Samsung 30Q and Sony/Murata VTC6 are all good. Two reasons:

- **Low internal resistance.** The motors pull short bursts of 1-3 A [E]. A good cell sags only a few tens of millivolts at that; a cheap one sags enough to change how hard the wheels push from one second to the next.
- **A real rating.** A 15-35 A continuous rating gives a large margin over the robot's 4 A worst case [E].

Buy them from 18650batterystore.com, imrbatteries.com, liionwholesale.com or Illumn, not from Amazon or eBay, where cells are often counterfeit or rewrapped. Never buy "UltraFire", "GTF" or anything claiming 9,900 mAh.

## Tools and consumables

Not counted in the price:

- Multimeter with a 10 A current range: to set both Mini360 bucks to 5.0 V before anything is connected, and for the bench measurements
- Soldering iron and solder, flush cutters, wire strippers, small Phillips screwdriver, 2.5 mm hex key, calipers
- Kitchen scale (mass, centre of mass, stall torque with a lever)
- A board you can tilt, covered with your demo surface (tyre friction)
- Phone with slow-motion video (servo speed)
- Two USB data cables (one per ESP32)
- CA glue for the encoder rings; a strip of bicycle inner tube for the tail foot
- Ender 3 or any FDM printer with a 220 x 220 mm bed, PLA and ABS
