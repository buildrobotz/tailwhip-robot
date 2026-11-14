"""Build the Tailwhip project website into site/dist/ (plain static files, relative links, no server code).

    python site/build_site.py
    cd site/dist && python -m http.server 8000      # preview: the 3D viewer needs http, not file://

Content comes from the repo itself: docs/*.md, docs/bom.json, docs/*.svg, cad/out/*, sim/eval.json, sim/out/demo.mp4
(python sim/demo.py) and the Blender animation (visual/, rendered outside the repo; pass --anim to point at it).
Edit those, rebuild, commit site/dist/; .github/workflows/pages.yml publishes it.
Layout and table handling follow the Ouro site (github.com/buildrobotz/ouro-robot).
"""
import argparse, json, re, shutil, html, pathlib
import markdown
from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC, DIST = ROOT / 'site' / 'src', ROOT / 'site' / 'dist'
REPO_URL = 'https://github.com/buildrobotz/tailwhip-robot'
DOMAIN = 'tailwhip.buildrobotz.com'
import os
ANIM = pathlib.Path(os.environ.get('TAILWHIP_ANIM', 'visual/out/tailwhip_demo.mp4')).expanduser()

PAGES = [  # slug, nav label, title, lede
    ('index', 'Overview', 'Tailwhip, the robot that sits on its tail',
     'A two-wheeled balancing robot built from the hobby parts in most robotics drawers. It rests on its tail, rears up onto its wheels, '
     'balances, looks around, and lands on its tail when you shove it. You train its brain with reinforcement learning '
     'in MuJoCo on your laptop, then run it on the real robot.'),
    ('build', 'Build plan', 'Build plan', 'From ordering parts to a robot that rears up on its own: seven milestones, each with a test that tells you it worked.'),
    ('parts', 'Parts', 'Parts and prices', 'Everything to buy for one robot, with what to look for in each listing and what you probably already own.'),
    ('mechanical', 'Mechanical', 'Print and assemble', 'Seven prints and one long bolt pair. Spin the models, download the STLs, and measure your TT motor before you print.'),
    ('electronics', 'Electronics', 'Circuit and wiring', 'One ESP32, two TT motors on an MX1508, two servos, an MPU6050 and a 2S 18650 pack. Schematic, wiring table and a safe bring-up.'),
    ('software', 'Software', 'Software', 'Firmware on the robot, a radio bridge on your laptop, and Python tools for driving, bench tests and running the policy.'),
    ('learning', 'Simulation and RL', 'Simulate and train', 'The MuJoCo model built from the CAD, every sloppy part made sloppy on purpose, what the policy sees and is rewarded for, and how to train your own.'),
    ('course', 'Learning path', 'Learning path', 'One-hour sessions from the simulator to a robot that gets back up. One thing to do, one thing to learn and one check in each.'),
    ('risks', 'Limits', 'Limits and gotchas', 'What to know before you order parts: what the design cannot do, what may break, and how to keep the battery safe.'),
]
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
    nav = ''.join(f'<a href="{s}.html"{cur if s == slug else ""}>{l}</a>' for s, l, _, _ in PAGES)
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
{toc(body)}{fold_reference(body)}
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
    sim_v = copy_file(ROOT / 'sim' / 'out' / 'demo.mp4', 'assets/video/sim_demo.mp4')
    anim_v = copy_file(ANIM, 'assets/video/design_animation.mp4')
    sim_poster = copy_img(ROOT / 'sim' / 'out' / 'demo_poster.png', 'sim_poster', 960)
    anim_poster = copy_img(ROOT / 'visual' / 'out' / 'anim_poster.png', 'anim_poster', 960)
    vids = (video(sim_v, sim_poster, 'The trained policy in MuJoCo, on the simulated robot with its sloppy gearboxes, coarse encoders and radio delay. '
                  'Rest, rear up, balance, drive, turn, take two shoves, lie back down. Nothing is scripted except the commands.', 'Simulation, trained policy')
            + video(anim_v, anim_poster, 'The design, animated in Blender from the CAD: what the finished robot should look like. Kinematic, not physics: the timing is hand-chosen.', 'Design animation'))
    facts = json.loads((ROOT / 'site' / 'facts.json').read_text())
    cards = ''.join(f'<a href="{s}.html"><b>{l}</b><span>{d}</span></a>' for s, l, _, d in PAGES[1:])
    return f'''
<section class="videos">{vids}</section>
<dl class="facts">{''.join(f"<div><dt>{k}</dt><dd>{v}</dd></div>" for k, v in facts.items())}</dl>
{status_note()}
<section class="prose">{md(doc('overview.md'))}</section>
<section><div class="prose"><h2>Build it, page by page</h2></div><div class="cards" style="margin-top:12px">{cards}</div></section>'''

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--anim', help='path to the Blender animation mp4 (or set TAILWHIP_ANIM); without it the published copy in dist/ is kept')
    args = ap.parse_args()
    global ANIM
    if args.anim:
        ANIM = pathlib.Path(args.anim).expanduser()
    keep_video = {}
    if DIST.exists():                       # keep already-published videos if their sources are not on this machine
        for f in (DIST / 'assets' / 'video').glob('*.mp4') if (DIST / 'assets' / 'video').exists() else []:
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
        r = globals()['p_' + slug]()
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
