Every bought part for one Tailwhip is in the table above, including the parts many builders already have (marked "Often already in a robotics parts drawer"), so that you can build one from this page. The list is AI-generated and unbuilt: check each listing before you order.

## What it costs

Most of these parts come in packs, so the price per robot depends on whether you share the packs.

- **If you already have the common parts**, the five extra purchases (two MG90S, the 2S BMS, the 18650 holder, the power switch, the polyfuse): **about $15 per robot** at the per-unit price of shared packs, or **about $38** buying each Amazon pack once. The cheapest single units (Waveshare servos, ShillehTek BMS, SparkFun holder, Pololu switch, DigiKey fuse) come to about $19 plus shipping.
- **Everything marked BUY** (adds the charger, charge jack, capacitor, resistors, screws, wire, connectors, perfboard and heat-shrink): about **$54 per robot** with packs shared, about **$172** buying every pack once. Most of that is kits (an M3 screw kit, heat-shrink, wire) that last for many robots.
- **A complete robot from nothing**, including the common parts: about **$114** from Amazon packs shared across builds, about **$107** taking the cheapest source per line.

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
