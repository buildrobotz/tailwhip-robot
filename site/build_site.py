"""Build the Tailwhip project website into site/dist/ (plain static files, relative links, no server code).

    python site/build_site.py
    cd site/dist && python -m http.server 8000      # preview: the 3D viewer needs http, not file://

Content comes from the repo itself: docs/*.md, docs/bom.json, docs/*.svg, cad/out/*, sim/eval.json, sim/out/demo.mp4
(python sim/demo.py) and the Blender animation (visual/, rendered outside the repo; pass --anim to point at it).
The four Stage 1 pages come from stage1/ (README.md, docs/*.md, docs/*.svg, docs/electronics.json, out/*, the firmware)
and an optional simulation clip (rendered outside the repo; pass --stage1-video to point at it).
Edit those, rebuild, commit site/dist/; .github/workflows/pages.yml publishes it.
Layout and table handling follow the Ouro site (github.com/buildrobotz/ouro-robot).
"""
import argparse, json, re, shutil, html, pathlib, subprocess
import markdown
from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC, DIST = ROOT / 'site' / 'src', ROOT / 'site' / 'dist'
REPO_URL = 'https://github.com/buildrobotz/tailwhip-robot'
DOMAIN = 'tailwhip.buildrobotz.com'
import os
ANIM = pathlib.Path(os.environ.get('TAILWHIP_ANIM', 'visual/out/tailwhip_demo.mp4')).expanduser()
STAGE1 = ROOT / 'stage1'
S1_VIDEO = os.environ.get('TAILWHIP_STAGE1_VIDEO')      # the stage 1 clip; without it the published copy in dist/ is kept
FFMPEG = '/opt/homebrew/bin/ffmpeg'                     # explicit: another ffmpeg on PATH is broken on the Mac

PAGES = [  # slug, nav label, title, lede
    ('index', 'Overview', 'Tailwhip, the robot that sits on its tail',
     'A two-wheeled balancing robot built from the hobby parts in most robotics drawers. It rests on its tail, rears up onto its wheels, '
     'balances, looks around, and lands on its tail when you shove it. You train its brain with reinforcement learning '
     'in MuJoCo on your laptop, then run it on the real robot.'),
    ('stage1', 'Stage 1', 'Stage 1: the bare-minimum balancer',
     'Build this first. One TT motor with a wheel on each end of its shaft, two 18650s, an ESP32, an IMU and a motor driver, '
     'kept upright by a PID loop. Its one job is to show that a TT motor is fast enough to balance a body.'),
    ('stage1-print', 'Stage 1: print', 'Stage 1: print and assemble',
     'Four PLA prints, no supports. Spin the models, download the STLs, measure your motor and boards before you print, then assemble it.'),
    ('stage1-wiring', 'Stage 1: wiring', 'Stage 1: circuit and wiring',
     'Every pin of every board and every wire, end to end: the schematic, the wire-by-wire drawing, the IMU wiring, and a power-up checklist with a meter reading at each step.'),
    ('stage1-firmware', 'Stage 1: tune', 'Stage 1: flash, bring up and tune',
     'Flash the PID firmware, check the IMU and the motor direction, tune the gains over serial, and test it against the pass criteria.'),
    ('build', 'Build plan', 'Build plan', 'From ordering parts to a robot that rears up on its own: seven milestones, each with a test that tells you it worked.'),
    ('parts', 'Parts', 'Parts and prices', 'Everything to buy for one robot, with what to look for in each listing and what you probably already own.'),
    ('mechanical', 'Mechanical', 'Print and assemble', 'Seven prints and one long bolt pair. Spin the models, download the STLs, and measure your TT motor before you print.'),
    ('electronics', 'Electronics', 'Circuit and wiring', 'One ESP32, two TT motors on an MX1508, two servos, an MPU6050 and a 2S 18650 pack. Schematic, wiring table and a safe bring-up.'),
    ('software', 'Software', 'Software', 'Firmware on the robot, a radio bridge on your laptop, and Python tools for driving, bench tests and running the policy.'),
    ('learning', 'Simulation and RL', 'Simulate and train', 'The MuJoCo model built from the CAD, every sloppy part made sloppy on purpose, what the policy sees and is rewarded for, and how to train your own.'),
    ('course', 'Learning path', 'Learning path', 'One-hour sessions from the simulator to a robot that gets back up. One thing to do, one thing to learn and one check in each.'),
    ('risks', 'Limits', 'Limits and gotchas', 'What to know before you order parts: what the design cannot do, what may break, and how to keep the battery safe.'),
]
# Stage 1 is one entry in the top navigation; its four pages share a second row of links.
S1_PAGES = [('stage1', 'Why and results'), ('stage1-print', 'Print and assemble'), ('stage1-wiring', 'Circuit and wiring'), ('stage1-firmware', 'Flash and tune')]
NAV_PARENT = {s: 'stage1' for s, _ in S1_PAGES}
MD = dict(extensions=['tables', 'fenced_code', 'sane_lists', 'toc'])


def label_table(t):
    heads = [re.sub(r'<[^>]+>', '', h).strip() for h in re.findall(r'<th[^>]*>(.*?)</th>', t, re.S)]
    if len(heads) < 3:
        return t
    def row(m):
        cells = iter(heads)
        return re.sub(r'<td(?=[\s>])', lambda c: f'<td data-label="{html.escape(next(cells, ""), quote=True)}"', m.group(0))
    t = re.sub(r'<tr>.*?</tr>', row, t, flags=re.S)
    return t.replace('<table', '<table class="stack"', 1)

def tables(out):
    out = re.sub(r'<table>.*?</table>', lambda m: label_table(m.group(0)), out, flags=re.S)
    return re.sub(r'<table', '<div class="tablewrap"><table', out).replace('</table>', '</table></div>')

def md(text):
    return tables(markdown.markdown(text, **MD))

def fold_reference(body):
    m = re.search(r'<h2 id="(?:design-)?reference[^"]*">.*?</h2>', body, re.S)
    if not m:
        return body
    start = m.end(); nxt = re.search(r'<h2[ >]', body[start:]); end = start + nxt.start() if nxt else len(body)
    parts = re.split(r'(<h3 id="[^"]+">.*?</h3>)', body[start:end], flags=re.S)
    out = parts[0]
    for i in range(1, len(parts), 2):
        title = re.sub(r'<[^>]+>', '', parts[i]); hid = re.search(r'id="([^"]+)"', parts[i]).group(1)
        out += f'<details class="ref" id="{hid}"><summary>{title}</summary>{parts[i + 1]}</details>'
    return body[:start] + out + body[end:]

def toc(body):
    heads = re.findall(r'<h2 id="([^"]+)">(.*?)</h2>', body, re.S)
    if len(heads) < 4:
        return ''
    items = ''.join(f'<li><a href="#{i}">{re.sub(r"<[^>]+>", "", t)}</a></li>' for i, t in heads)
    return f'<details class="toc"><summary>On this page</summary><ol>{items}</ol></details>'

def doc(name, drop_title=True):
    p = ROOT / 'docs' / name
    if not p.exists():
        return ''
    t = p.read_text()
    t = re.sub(r'\A# [^\n]*\n', '', t) if drop_title else t
    return t

def page(slug, body, scripts=''):
    i = [p[0] for p in PAGES].index(slug); _, label, title, lede = PAGES[i]
    brz = 'https://buildrobotz.com'
    cur = ' aria-current="page"'
    here = NAV_PARENT.get(slug, slug)
    nav = ''.join(f'<a href="{s}.html"{cur if s == here else ""}>{l}</a>'
                  for s, l, _, _ in PAGES if NAV_PARENT.get(s, s) == s)
    sub = ''
    if slug in NAV_PARENT:
        sub = ('<nav class="subnav" aria-label="Stage 1 pages">'
               + ''.join(f'<a href="{s}.html"{cur if s == slug else ""}>{n}. {l}</a>' for n, (s, l) in enumerate(S1_PAGES, 1)) + '</nav>')
    prev = f'<a href="{PAGES[i-1][0]}.html">&larr; {PAGES[i-1][1]}</a>' if i else '<span></span>'
    nxt = f'<a href="{PAGES[i+1][0]}.html">{PAGES[i+1][1]} &rarr;</a>' if i + 1 < len(PAGES) else '<span></span>'
    return f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{html.escape(label if slug != "index" else "Tailwhip")} · Tailwhip robot</title>
<meta name="description" content="{html.escape(lede)}">
<meta property="og:title" content="{html.escape(title)}"><meta property="og:description" content="{html.escape(lede)}">
<meta property="og:image" content="https://{DOMAIN}/assets/cad_render_upright.jpg">
<link rel="icon" href="data:,"><link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Bricolage+Grotesque:opsz,wght@12..96,500;12..96,800&family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<link rel="stylesheet" href="assets/style.css"></head>
<body>
<div class="crumb"><div class="wrap"><a href="{brz}/">BuildRobotz</a><span aria-hidden="true">/</span><a href="{brz}/robots/">Robots</a><span aria-hidden="true">/</span><span>Tailwhip</span></div></div>
<header class="top"><div class="wrap"><a class="brand" href="index.html">Tailwhip<span>.</span></a><nav aria-label="Project pages">{nav}</nav></div></header>
<div class="wrap"><main>
<header><div class="eyebrow">BuildRobotz project · {html.escape(label)}</div><h1>{html.escape(title)}</h1><p class="lede">{html.escape(lede)}</p></header>
{sub}{toc(body)}{fold_reference(body)}
<div class="pager">{prev}{nxt}</div>
</main></div>
<footer><div class="wrap">Tailwhip is an open robot project from <a href="{brz}/">BuildRobotz</a>. The design was generated with AI and has not yet been built or tested on hardware; performance figures are MuJoCo simulation results. Build at your own risk. Code, CAD and firmware: <a href="{REPO_URL}">{REPO_URL.split("//")[1]}</a> (code MIT, docs and CAD CC BY-SA 4.0).<br>
<a href="{brz}/robots/">More robots to build</a> · <a href="https://ouro.buildrobotz.com/">Ouro, its sister project</a> · <a href="{brz}/">Learn robotics with BuildRobotz</a></div></footer>
<script>(function(){{var a=document.querySelector('nav [aria-current]');if(a&&a.parentNode.scrollWidth>a.parentNode.clientWidth)a.parentNode.scrollLeft=a.offsetLeft-a.parentNode.clientWidth/2+a.offsetWidth/2;
function op(){{var t=location.hash&&document.getElementById(decodeURIComponent(location.hash.slice(1)));for(var e=t;e;e=e.parentElement)if(e.tagName==='DETAILS')e.open=true;if(t)t.scrollIntoView();}}
addEventListener('hashchange',op);op();}})();</script>
{scripts}
</body></html>'''

def copy_img(src, name, w=1100):
    src = pathlib.Path(src)
    if not src.exists():
        return None
    im = Image.open(src).convert('RGB')
    if im.width > w:
        im = im.resize((w, int(im.height * w / im.width)))
    im.save(DIST / 'assets' / (name + '.jpg'), 'JPEG', quality=82)
    return f'assets/{name}.jpg'

def copy_file(src, dst):
    src = pathlib.Path(src)
    if not src.exists():
        return None
    (DIST / dst).parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(src, DIST / dst)
    return dst

def published(dst):
    """A file kept from the previous build (videos whose sources are not on this machine)."""
    return dst if (DIST / dst).exists() else None

def fig(url, cap, alt=None):
    return f'<figure class="fig"><img src="{url}" alt="{html.escape(alt or cap)}" loading="lazy"><figcaption>{cap}</figcaption></figure>' if url else ''

def video(url, poster, cap, label):
    if not url:
        return ''
    p = f' poster="{poster}"' if poster else ''
    return (f'<figure><video src="{url}"{p} controls muted loop playsinline preload="metadata"></video>'
            f'<figcaption><b>{label}</b><br>{cap}</figcaption></figure>')

def status_note():
    return ('<div class="note"><b>Read this first: AI-generated, not yet built</b>This design, its code and these pages were generated with AI, and nobody has built or tested a physical Tailwhip yet. '
            'The CAD, simulator, trained policy, firmware and print files are complete and checked numerically, and every performance number here comes from simulation. '
            'Treat it as a well-worked starting point: measure the parts the pages tell you to, check each step as you go, expect to fix things, and build at your own risk, especially around the 18650 cells. '
            'The <a href="risks.html">Limits</a> page lists what to measure and watch.</div>')

def eval_facts():
    p = ROOT / 'sim' / 'eval.json'
    return json.loads(p.read_text()) if p.exists() else None


# ------------------------------------------------------------------------------------------- pages
def p_index():
    sim_v = copy_file(ROOT / 'sim' / 'out' / 'demo.mp4', 'assets/video/sim_demo.mp4') or published('assets/video/sim_demo.mp4')
    anim_v = copy_file(ANIM, 'assets/video/design_animation.mp4') or published('assets/video/design_animation.mp4')
    sim_poster = copy_img(ROOT / 'sim' / 'out' / 'demo_poster.png', 'sim_poster', 960)
    anim_poster = copy_img(ROOT / 'visual' / 'out' / 'anim_poster.png', 'anim_poster', 960)
    vids = (video(sim_v, sim_poster, 'The trained policy in MuJoCo, on the simulated robot with its sloppy gearboxes, coarse encoders and radio delay. '
                  'Rest, rear up, balance, drive, turn, take two shoves, lie back down. Nothing is scripted except the commands.', 'Simulation, trained policy')
            + video(anim_v, anim_poster, 'The design, animated in Blender from the CAD: what the finished robot should look like. Kinematic, not physics: the timing is hand-chosen.', 'Design animation'))
    facts = json.loads((ROOT / 'site' / 'facts.json').read_text())
    def cards(pages):
        return ''.join(f'<a href="{s}.html"><b>{t if s in NAV_PARENT else l}</b><span>{d}</span></a>' for s, l, t, d in pages)
    s1 = [p for p in PAGES if p[0] in NAV_PARENT]; rest = [p for p in PAGES[1:] if p[0] not in NAV_PARENT]
    return f'''
<section class="videos">{vids}</section>
<dl class="facts">{''.join(f"<div><dt>{k}</dt><dd>{v}</dd></div>" for k, v in facts.items())}</dl>
{status_note()}
<section class="prose">{md(doc('overview.md'))}</section>
<section><div class="prose"><h2 id="build-it-page-by-page">Build it, page by page</h2>
<h4>First: stage 1, the bare-minimum balancer</h4></div><div class="cards" style="margin-top:12px">{cards(s1)}</div>
<div class="prose"><h4>Then: the full robot</h4></div><div class="cards" style="margin-top:12px">{cards(rest)}</div></section>'''

def money(x):
    return '' if x is None else f'${x:,.2f}'

GROUPS = [('core', 'Core robot', 'Everything the robot needs to rear up and balance.'),
          ('optional', 'Optional', 'The camera head (milestone 6) and spare metal-gear servos.'),
          ('shop', 'Shop tools', 'Bought once, used for many robots. Skip what you already have.')]

def bom_row(it):
    q = it.get('qty_per_robot_num', it.get('qty_per_robot'))
    a, alts = it.get('amazon') or {}, [x for x in (it.get('alt'), it.get('alt2')) if x]
    qs = re.match(r'[\d.]+', str(q))
    qn = float(q) if isinstance(q, (int, float)) else (float(qs.group()) if qs else 1.0)
    au = a.get('unit_usd'); best = min([x for x in alts if x.get('unit_usd') is not None], key=lambda x: x['unit_usd'], default=None)
    line = au * qn if au is not None else (best['unit_usd'] * qn if best else None)
    cheapest = min([v for v in (au, best and best['unit_usd']) if v is not None], default=None)
    acell = (f'<a href="{a["url"]}" rel="noopener">{html.escape((a.get("title") or "Amazon listing")[:70])}</a><br>'
             f'{(("about " + money(a.get("price_usd"))) if a.get("price_usd") else "")}{(" / pack of " + str(a["pack_qty"])) if a.get("price_usd") and a.get("pack_qty") else ""}') if a.get('url') else (
             f'about {money(a.get("price_usd"))}' if a.get('price_usd') else '&ndash;')
    ccell = '<br>'.join((f'<a href="{x["url"]}" rel="noopener">{html.escape(x.get("vendor", "Vendor"))}</a>' if x.get('url') else html.escape(x.get('vendor', 'Vendor')))
                        + f': {money(x.get("price_usd"))}{(" / " + str(x["pack_qty"])) if x.get("pack_qty") else ""}' for x in alts) or '&ndash;'
    note = html.escape(it.get('build_note') or a.get('notes') or '')
    row = (f'<tr><td data-label="#">{it.get("id", "")}</td><td data-label="Part" class="lead"><b>{html.escape(it["name"])}</b><br><span style="color:var(--muted);font-size:.85rem">{note}</span></td>'
           f'<td class="num" data-label="Per robot">{html.escape(str(q))}</td><td data-label="Amazon">{acell}</td><td data-label="Other sources">{ccell}</td><td class="num" data-label="Line">{money(line)}</td></tr>')
    return row, (line or 0.0), (cheapest or 0.0) * qn

def p_parts():
    bom = json.loads((ROOT / 'docs' / 'bom.json').read_text())
    tables, totals = '', {}
    for g, label, blurb in GROUPS:
        items = [it for it in bom if it.get('group', 'core') == g]
        rows = [bom_row(it) for it in items]
        totals[g] = (sum(r[1] for r in rows), sum(r[2] for r in rows))
        tables += (f'<section class="prose"><h2 id="{g}">{label}: about {money(totals[g][0])} per robot</h2><p>{blurb}</p></section>'
                   f'<div class="tablewrap wide"><table class="stack"><thead><tr><th>#</th><th>Part</th><th class="num">Per robot</th><th>Amazon</th>'
                   f'<th>Other sources</th><th class="num">Line</th></tr></thead><tbody>{"".join(r[0] for r in rows)}</tbody></table></div>')
    return f'''
<div class="note"><b>About the prices</b>Prices are typical listing prices and they move, so click through and check before you order. "Per robot" counts only what one robot uses, at the per-unit price inside each pack; packs leave you spares. Many builders already own the motors, servos and ESP32s: the table says so.</div>
<dl class="facts"><div><dt>Core robot</dt><dd>{money(totals['core'][0])}</dd></div><div><dt>Core, cheapest source per line</dt><dd>{money(totals['core'][1])}</dd></div>
<div><dt>Optional</dt><dd>{money(totals['optional'][0])}</dd></div><div><dt>Shop tools</dt><dd>{money(totals['shop'][0])}</dd></div></dl>
<section class="prose">{md(doc('parts.md'))}</section>
{tables}
{('<section class="prose"><h2>Sourcing notes</h2>' + md(doc('bom-sources.md')) + '</section>') if doc('bom-sources.md') else ''}'''

def p_mechanical():
    out = ROOT / 'cad' / 'out'
    models = []
    for label, n, color in (('Balancing', 'assembled_upright.glb', None), ('Resting on the tail', 'assembled_rest.glb', None),
                            ('Core, left half', 'print/core_left.stl', '#e8862a'), ('Core, right half', 'print/core_right.stl', '#e8862a'),
                            ('Torso', 'print/torso.stl', '#f2b84b'), ('Tail (ABS)', 'print/tail.stl', '#2a9ea8'),
                            ('Head', 'print/head.stl', '#e9e4d8'), ('Encoder ring (x2)', 'print/encoder_ring.stl', '#cfcfc8')):
        if (out / n).exists():
            dst = 'models/' + pathlib.Path(n).name
            copy_file(out / n, dst); models.append(dict(label=label, file=dst, color=color))
    CAPTION = {'render_upright': 'Balancing, tail held ready at its rest angle', 'render_rest': 'At rest on its tail, 25 deg back',
               'render_back': 'From behind: ESP32 on the spine, tail servo in the hip block', 'render_front': 'From the front: power boards on the spine, IMU on the motor web, feet',
               'render_parts': 'The printed parts', 'side_upright': 'Side view with the main dimensions', 'side_rest': 'Resting on the tail',
               'side_heel': 'A backward fall ends on the tail', 'side_chin': 'A forward fall ends on the feet'}
    gallery = ''.join(fig(copy_img(out / f'{k}.png', 'cad_' + k), v) for k, v in CAPTION.items() if (out / f'{k}.png').exists())
    dl = ''.join(f'<a class="btn" href="models/{p.name}">{p.name}</a> ' for p in sorted((DIST / 'models').glob('*.stl')))
    if (out / 'analysis.md').exists():
        copy_file(out / 'analysis.md', 'assets/analysis.md'); dl += '<a class="btn" href="assets/analysis.md">analysis.md (every design check)</a>'
    return f'''
<section><div class="viewer" data-models='{json.dumps(models)}'><canvas class="view"></canvas><span class="hint">drag to orbit · scroll to zoom</span></div>
<div class="bar vbar" style="border:1px solid var(--rule);border-top:0;border-radius:0 0 6px 6px"></div></section>
<section class="prose"><h4>Download the print files</h4><p>{dl}</p></section>
<div class="gallery">{gallery}</div>
<section class="prose">{md(doc('mechanical.md'))}</section>''', '''<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/GLTFLoader.js"></script>
<script src="assets/viewer.js"></script>'''

def p_electronics():
    svgs = ''
    for f, cap in (('schematic.svg', 'Schematic'), ('wiring.svg', 'Where every board sits'), ('perfboard.svg', 'Power board layout')):
        if (ROOT / 'docs' / f).exists():
            copy_file(ROOT / 'docs' / f, f'assets/{f}')
            svgs += f'<figure class="fig"><div class="svgwrap"><a href="assets/{f}"><img src="assets/{f}" alt="{cap}"></a></div><figcaption><b>{cap.upper()}</b> · open the image for full size</figcaption></figure>'
    copy_file(ROOT / 'docs' / 'electronics.json', 'assets/electronics.json')
    body = re.sub(r'!\[[^\]]*\]\((?:\./)?(schematic|wiring|perfboard)\.svg\)', '', doc('electronics.md'))
    return f'{svgs}<section class="prose">{md(body)}</section>'

def p_software():
    return f'<section class="prose">{md(doc("software.md"))}</section>'

def p_learning():
    return f'<section class="prose">{md(doc("simulation.md"))}</section>'

def p_build():
    return f'{status_note()}<section class="prose">{md(doc("build.md"))}</section>'

def p_course():
    return f'<section class="prose">{md(doc("learning-path.md"))}</section>'

def p_risks():
    return f'<section class="prose">{md(doc("limits.md"))}</section>'


# ------------------------------------------------------------------------------------------- stage 1
# Everything below is read from stage1/ at build time; nothing is copied into this script except page structure.
# Stage 1 files on the site: where a link or a `path` in the stage 1 docs should point.
S1_SITE = {'stage1/README.md': 'stage1.html', 'stage1/docs/mechanical.md': 'stage1-print.html',
           'stage1/docs/electronics.md': 'stage1-wiring.html', 'stage1/docs/schematic.svg': 'assets/stage1_schematic.svg',
           'stage1/docs/wiring.svg': 'assets/stage1_wiring.svg', 'stage1/docs/electronics.json': 'assets/stage1_electronics.json',
           'docs/electronics.md': 'electronics.html', 'docs/mechanical.md': 'mechanical.html', 'docs/build.md': 'build.html',
           'docs/parts.md': 'parts.html', 'docs/software.md': 'software.html', 'docs/limits.md': 'risks.html'}
S1_FW = 'stage1/firmware/stage1_balance/stage1_balance.ino'

def s1_read(rel):
    p = STAGE1 / rel
    if not p.exists():
        raise SystemExit(f'stage 1: {p} is missing')
    return p.read_text()

def grab(pattern, text, what, flags=0):
    m = re.search(pattern, text, flags)
    if not m:
        raise SystemExit(f'stage 1: could not find {what} (pattern {pattern!r}); the stage 1 files changed shape')
    return m

def sections(text):
    """(preamble, [(title, body)]) split at '## ' headings, with the '# ' title dropped."""
    text = re.sub(r'\A# [^\n]*\n', '', text)
    parts = re.split(r'^## (.+)\n', text, flags=re.M)
    return parts[0], [(parts[i].strip(), parts[i + 1]) for i in range(1, len(parts), 2)]

def section(secs, title):
    for t, b in secs:
        if t.lower().startswith(title.lower()):
            return b
    raise SystemExit(f'stage 1: no "## {title}" section')

def reference(secs, titles=None, drop=()):
    """The named sections (all if titles is None) as one '## Reference' block: each folds into a <details> on the page."""
    keep = [(t, b) for t, b in secs if titles is None or any(t.lower().startswith(x.lower()) for x in titles)]
    if not keep:
        return ''
    out = '## Reference\n\n'
    for t, b in keep:
        b = re.sub(r'^(#{3,5}) ', r'#\1 ', b, flags=re.M)
        for d in drop:
            b = re.sub(r'!\[[^\]]*\]\(' + re.escape(d) + r'\)\n?', '', b)
        out += f'### {t}\n\n{b}\n'
    return out

def unref(secs, titles):
    return [(t, b) for t, b in secs if not any(t.lower().startswith(x.lower()) for x in titles)]

def joined(secs):
    return ''.join(f'## {t}\n{b}' for t, b in secs)

def s1_url(p):
    """Where the site should point for a repo file: a site page or asset if there is one, else the file on GitHub."""
    p = p.resolve()
    try:
        rel = p.relative_to(ROOT).as_posix()
    except ValueError:
        return None
    if rel in S1_SITE:
        return S1_SITE[rel]
    if rel.startswith('stage1/out/') and p.suffix == '.png' and p.exists():
        return copy_img(p, 'stage1_' + p.stem)
    if p.exists():
        return f'{REPO_URL}/{"tree" if p.is_dir() else "blob"}/main/{rel}'
    return None

def s1_find(path, base):
    """A path written in a stage 1 doc, resolved the way its authors use them: next to the doc, then from stage1/,
    then from the repo root. 'docs/...' inside stage1/docs means the full robot's docs."""
    here = STAGE1 / base
    cands = [here / path]
    if not (base and path.startswith('docs/')):
        cands.append(STAGE1 / path)
    cands.append(ROOT / path)
    for c in cands:
        if c.exists():
            return c
    return None

def s1_md(text, base, fixes=()):
    """Stage 1 markdown, ready for md(): images copied to assets/, links and `paths` pointed at the site or GitHub."""
    for old, new in fixes:
        text, n = re.subn(old, new, text)
        if not n:
            print(f'  note: stage 1 text not found, left as is: {old!r}')
    def img(m):
        alt, path = m.group(1), m.group(2)
        p = (STAGE1 / base / path).resolve()
        if p.suffix == '.png':
            url = copy_img(p, 'stage1_' + p.stem)
            return f'![{alt}]({url})' if url else ''
        return ''          # SVG diagrams are shown full size at the top of the wiring page
    text = re.sub(r'!\[([^\]]*)\]\(([^)\s]+)\)\n?', img, text)
    def link(m):
        label, path = m.group(1), m.group(2)
        if re.match(r'[a-z]+:|#', path):
            return m.group(0)
        url = s1_url(STAGE1 / base / path)
        return f'[{label}]({url})' if url else m.group(0)
    text = re.sub(r'(?<![!\]])\[([^\]]+)\]\(([^)\s]+)\)', link, text)
    def code(m):
        path = m.group(1)
        if not re.search(r'[./]', path):
            return m.group(0)
        found = s1_find(path, base)
        url = s1_url(found) if found else None
        return f'[`{path}`]({url})' if url else m.group(0)
    return re.sub(r'(?<![`\[])`([\w.][\w./-]*)`(?![`\]])', code, text)

def s1_note():
    st = grab(r'\*\*Status:\*\*\s*(.+?)\n\n', s1_read('README.md'), 'the README status line', re.S).group(1)
    st = re.sub(r'\s+', ' ', st).strip(); st = st[0].upper() + st[1:]
    st = re.sub(r'^<p>|</p>$', '', markdown.markdown(st))
    return ('<div class="note"><b>Read this first: designed and simulated, not built</b>' + st +
            ' Like the rest of Tailwhip, the design, its code and these pages were generated with AI. Measure what the pages tell you to, '
            'check each step with a meter, expect to fix things, and build at your own risk, especially around the 18650 cells.</div>')

def s1_pass():
    return grab(r'(\*\*Stage 1 passes when\*\*.+?)\n\n', s1_read('README.md'), 'the pass criteria', re.S).group(1)

def btns(items):
    return ' '.join(f'<a class="btn" href="{u}"{(" download=" + chr(34) + d + chr(34)) if isinstance(d, str) else " download" if d else ""}>{html.escape(l)}</a>'
                    for l, u, d in items if u)

def s1_video():
    """The simulation clip: re-encoded for the web from --stage1-video, else the copy already published."""
    dst, poster = 'assets/video/stage1_sim.mp4', 'assets/video/stage1_sim_poster.jpg'
    if S1_VIDEO:
        src = pathlib.Path(S1_VIDEO).expanduser()
        if not src.exists():
            raise SystemExit(f'--stage1-video: {src} not found')
        (DIST / 'assets' / 'video').mkdir(parents=True, exist_ok=True)
        subprocess.run([FFMPEG, '-y', '-loglevel', 'error', '-i', str(src), '-vf', 'scale=854:-2', '-r', '30', '-c:v', 'h264_videotoolbox',
                        '-b:v', '700k', '-pix_fmt', 'yuv420p', '-an', '-movflags', '+faststart', str(DIST / dst)], check=True)
        subprocess.run([FFMPEG, '-y', '-loglevel', 'error', '-ss', '1', '-i', str(src), '-frames:v', '1', '-vf', 'scale=960:-2',
                        '-q:v', '4', str(DIST / poster)], check=True)
    return published(dst), published(poster)

def s1_facts():
    design, readme = s1_read('out/design.md'), s1_read('README.md')
    res = json.loads(s1_read('out/results.json'))
    mass = grab(r'\*\*Total\*\* \| \| \*\*(\d+)\*\*', design, 'the total mass in out/design.md').group(1)
    tall = grab(r'(\d+) mm tall', design, 'the height in out/design.md').group(1)
    cost = grab(r'\*\*about (\$\d+)\*\*', readme, 'the parts total in README.md').group(1)
    hz = grab(r'(\d+) Hz loop', readme, 'the loop rate in README.md').group(1)
    rb = res['robust']
    return {'Wheels': '2 on one TT motor', 'Mass': f'about {mass} g', 'Height': f'{tall} mm', 'Parts': f'about {cost}',
            'Brain': f'PID at {hz} Hz', 'Stays up (sim)': f"{100 * rb['survived'] / rb['n']:.0f} %"}

def p_stage1():
    pre, secs = sections(s1_read('README.md'))
    intro = grab(r'\A\s*(.+?)\n\n', pre, 'the README intro', re.S).group(1)
    hero = copy_img(STAGE1 / 'out' / 'render_hero.png', 'stage1_render_hero')
    vid, poster = s1_video()
    clip = video(vid, poster, 'The nominal robot in MuJoCo running the firmware\'s filter and PID: released at 8 deg, then shoved forward and back '
                 '(0.05 N s each). The run plotted below.', 'Simulation, PID')
    pic = fig(hero, '<b>Design</b><br>Front: the IMU on the core; BMS, switch and junction board on the mast. Rendered from the CAD.', 'Stage 1, rendered from the CAD')
    sim_pre, sim_secs = sections(s1_read('out/sim.md'))
    facts = s1_facts()
    text = (f'## Why it comes first\n\n{intro}\n\n'
            'The full robot counts on its TT motors to catch every fall. Stage 1 tests that one assumption with as little else as possible, '
            f'so that if a TT motor is too slow or too weak, you find out with a robot that costs {facts["Parts"]} and four prints.\n\n'
            f'## The three stages\n{section(secs, "The three stages")}'
            f'## What the simulation says\n{section(secs, "What the simulation says")}'
            f'## Parts\n{section(secs, "Parts")}'
            f'## Risks\n{section(secs, "Risks")}'
            + reference([('Simulation results in full', sim_pre + re.sub(r'^## ', '### ', joined(sim_secs), flags=re.M))]
                        + [(t, b) for t, b in secs if t.lower().startswith('files')], drop=('plot_nominal.png', 'plot_limits.png', 'plot_sweeps.png')))
    return f'''{s1_note()}
<section class="videos">{clip}{pic}</section>
<dl class="facts">{''.join(f"<div><dt>{k}</dt><dd>{v}</dd></div>" for k, v in facts.items())}</dl>
<section class="prose">{md(s1_md(text, '', S1_README_FIXES))}</section>'''

# Pointers in stage1/README.md that mean something else on the site.
S1_README_FIXES = ((r'Full results: `out/sim\.md`\. Clip: `python sim\.py video [\w.]+`\.', 'Full results are under [Reference](#reference) at the bottom of this page.'),
                   (r'until bench step 3', 'until bring-up step 3 ([Flash and tune](stage1-firmware.html#bring-up))'))

S1_PRINTS = (('Core, left half', 'core_left', '#e8862a'), ('Core, right half', 'core_right', '#e8862a'),
             ('Mast', 'mast', '#f2b84b'), ('Tray', 'tray', '#f2b84b'))
S1_RENDERS = {'render_back': 'From behind: ESP32, MX1508 and buck on the mast; motor plug K1 on the core',
              'render_views': 'Side, fallen forward onto a crash bar, front and back',
              'render_parts': 'Exploded: core halves, pins and bolts, motor, mast with boards, tray, holder, cells'}

def p_stage1_print():
    out = STAGE1 / 'out'
    models = []
    for label, n, color in S1_PRINTS:
        if copy_file(out / 'print' / f'{n}.stl', f'models/stage1/{n}.stl'):
            models.append(dict(label=label, file=f'models/stage1/{n}.stl', color=color))
    if len(models) != len(S1_PRINTS):
        raise SystemExit('stage 1: a print STL is missing from stage1/out/print/')
    copy_file(out / 'design.md', 'assets/stage1_design.md')
    dl = btns([(m['file'].split('/')[-1], m['file'], 'tailwhip_stage1_' + m['file'].split('/')[-1]) for m in models]
              + [('design.md (every design check)', 'assets/stage1_design.md', False)])
    body = s1_read('docs/mechanical.md')
    shown = set(re.findall(r'\(\.\./out/(render_\w+)\.png\)', body))
    gallery = ''.join(fig(copy_img(out / f'{k}.png', 'stage1_' + k), v) for k, v in S1_RENDERS.items() if k not in shown and (out / f'{k}.png').exists())
    pre, secs = sections(body)
    moved = ('Files', 'What changed from the first pass')
    text = pre + joined(unref(secs, moved)) + reference(secs, moved)
    return f'''{s1_note()}
<section><div class="viewer" data-models='{json.dumps(models)}'><canvas class="view"></canvas><span class="hint">drag to orbit · scroll to zoom</span></div>
<div class="bar vbar" style="border:1px solid var(--rule);border-top:0;border-radius:0 0 6px 6px"></div></section>
<section class="prose"><h4>Download the print files</h4><p>{dl}</p><p>Each STL is in its print orientation, lying on the bed. All four fit a 220 x 220 mm bed.</p></section>
<div class="gallery">{gallery}</div>
<section class="prose">{md(s1_md(text, 'docs'))}</section>''', '''<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/GLTFLoader.js"></script>
<script src="assets/viewer.js"></script>'''

def s1_svg(name, cap):
    src = STAGE1 / 'docs' / name
    s = s1_read('docs/' + name)
    w, h = grab(r"<svg[^>]*?width=['\"](\d+)['\"][^>]*?height=['\"](\d+)['\"]", s, f'the size of {name}').groups()
    url = copy_file(src, f'assets/stage1_{name}')
    return (f'<figure class="fig"><div class="svgwrap native"><a href="{url}"><img src="{url}" alt="{html.escape(cap)}" width="{w}" height="{h}"></a></div>'
            f'<figcaption><b>{cap.upper()}</b> · full size: scroll inside the frame, or open the image on its own</figcaption></figure>')

def p_stage1_wiring():
    figs = s1_svg('schematic.svg', 'Schematic') + s1_svg('wiring.svg', 'Every wire, on the robot')
    copy_file(STAGE1 / 'docs' / 'electronics.json', 'assets/stage1_electronics.json')
    dl = btns([('schematic.svg', 'assets/stage1_schematic.svg', 'tailwhip_stage1_schematic.svg'),
               ('wiring.svg', 'assets/stage1_wiring.svg', 'tailwhip_stage1_wiring.svg'),
               ('electronics.json', 'assets/stage1_electronics.json', 'tailwhip_stage1_electronics.json'),
               ('gen_electronics.py (makes all three)', s1_url(STAGE1 / 'docs' / 'tools' / 'gen_electronics.py'), False)])
    pre, secs = sections(s1_read('docs/electronics.md'))
    moved = ('What changed from the first-pass wiring', 'Firmware notes', 'Sources for the numbers')
    fixes = ((r"(?:carry on with )?the README's bring-up", 'carry on with the [bring-up on the Flash and tune page](stage1-firmware.html#bring-up)'),
             (r"from the README's bench step 3", 'from bring-up step 3 ([Flash and tune](stage1-firmware.html#bring-up))'))
    text = pre + joined(unref(secs, moved)) + reference(secs, moved)
    return f'''{s1_note()}
{figs}
<section class="prose"><h4>Download the drawings</h4><p>{dl}</p></section>
<section class="prose">{md(s1_md(text, 'docs', fixes))}</section>'''

def p_stage1_firmware():
    _, secs = sections(s1_read('README.md'))
    ino = s1_read('firmware/stage1_balance/stage1_balance.ino')
    helptext = grab(r'void help\(\)\s*\{(.*?)\n\}', ino, 'help() in the firmware', re.S).group(1)
    cmds = '\n'.join(s.replace('\\n', '') for s in re.findall(r'"((?:[^"\\]|\\.)*)"', helptext))
    dl = btns([('stage1_balance.ino on GitHub', f'{REPO_URL}/blob/main/{S1_FW}', False),
               ('Download the sketch', f'{REPO_URL}/raw/main/{S1_FW}', False)])
    text = f'''## The firmware

{section(secs, "Firmware")}
## Flash it

Use the same toolchain as the full robot ([its setup](electronics.html#set-up-the-firmware-toolchain)): the Arduino IDE or
`arduino-cli` with the esp32 core, board **ESP32 Dev Module**. The sketch needs no extra libraries. It has been checked to
compile with `arduino-cli` and esp32 core 3.3.12. From the top of the repository:

```bash
arduino-cli compile -b esp32:esp32:esp32 stage1/firmware/stage1_balance
arduino-cli upload  -b esp32:esp32:esp32 -p /dev/cu.usbserial-XXXX stage1/firmware/stage1_balance
arduino-cli monitor -p /dev/cu.usbserial-XXXX -c baudrate=115200
```

Type `help` in the serial monitor for the commands:

```
{cmds}
```

## Bring-up

Do the [power-up checklist](stage1-wiring.html#power-up-checklist) on the Circuit and wiring page first: it flashes this sketch at
step 0 and checks the IMU and the motor one connection at a time. Then:

{section(secs, "Bring-up")}
## Pass criteria

{s1_pass()}

## After stage 1

Stage 2 adds a second TT motor, wheel sensors and a bar that holds the body at a lean (see [the three stages](stage1.html#the-three-stages));
it is planned but not designed yet. Stage 1's core, electronics and firmware carry into it. Until then, the next step is the full robot:
start at the [Build plan](build.html).
'''
    return f'''{s1_note()}
<section class="prose"><h4>The sketch</h4><p>{dl}</p></section>
<section class="prose">{md(s1_md(text, ''))}</section>'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--anim', help='path to the Blender animation mp4 (or set TAILWHIP_ANIM); without it the published copy in dist/ is kept')
    ap.add_argument('--stage1-video', help='path to the stage 1 simulation clip, the nominal run of stage1/sim.py (or set TAILWHIP_STAGE1_VIDEO); '
                    'it is re-encoded for the web with ' + FFMPEG + '; without it the published copy in dist/ is kept')
    args = ap.parse_args()
    global ANIM, S1_VIDEO
    if args.anim:
        ANIM = pathlib.Path(args.anim).expanduser()
    if args.stage1_video:
        S1_VIDEO = args.stage1_video
    keep_video = {}
    if DIST.exists():                       # keep already-published videos (and their posters) if their sources are not on this machine
        for f in (DIST / 'assets' / 'video').glob('*') if (DIST / 'assets' / 'video').exists() else []:
            keep_video[f.name] = f.read_bytes()
        shutil.rmtree(DIST)
    (DIST / 'assets').mkdir(parents=True)
    (DIST / 'models').mkdir()
    for f in ('style.css', 'viewer.js'):
        shutil.copy(SRC / f, DIST / 'assets' / f)
    for name, data in keep_video.items():
        (DIST / 'assets' / 'video').mkdir(exist_ok=True)
        (DIST / 'assets' / 'video' / name).write_bytes(data)
    for slug, *_ in PAGES:
        r = globals()['p_' + slug.replace('-', '_')]()
        body, scripts = r if isinstance(r, tuple) else (r, '')
        body = re.sub(r'(src|href)="(?:\./)?(schematic\.svg|wiring\.svg|perfboard\.svg|electronics\.json)"', r'\1="assets/\2"', body)
        body = re.sub(r'(src|href)="\.\./cad/out/([\w-]+)\.png"', r'\1="assets/cad_\2.jpg"', body)
        for m in re.findall(r'src="\.\./cad/out/([\w-]+)\.png"', body):
            pass
        (DIST / f'{slug}.html').write_text(page(slug, body, scripts))
    # images referenced from the docs as ../cad/out/*.png
    for m in set(re.findall(r'assets/cad_([\w-]+)\.jpg', ''.join(p.read_text() for p in DIST.glob('*.html')))):
        if not (DIST / 'assets' / f'cad_{m}.jpg').exists():
            copy_img(ROOT / 'cad' / 'out' / f'{m}.png', f'cad_{m}')
    (DIST / 'CNAME').write_text(DOMAIN + '\n')
    (DIST / '.nojekyll').write_text('')
    n = sum(1 for _ in DIST.rglob('*') if _.is_file()); size = sum(f.stat().st_size for f in DIST.rglob('*') if f.is_file())
    print(f'built {len(PAGES)} pages, {n} files, {size / 1e6:.1f} MB -> {DIST}')

if __name__ == '__main__':
    main()
