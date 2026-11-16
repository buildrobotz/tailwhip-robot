"""Build the Tailwhip project website into site/dist/ (plain static files, relative links, no server code).

    python site/build_site.py
    cd site/dist && python -m http.server 8000      # preview: the 3D viewer needs http, not file://

Content comes from the repo itself: docs/*.md, docs/bom.json, docs/*.svg, cad/out/*, sim/eval.json, sim/out/demo.mp4
(python sim/demo.py) and the Blender animation (visual/, rendered outside the repo; pass --anim to point at it).
Each stageN/GUIDE.md becomes a step-by-step guide page plus the reference pages it lists, with every number, table,
picture and diagram pulled from that stage's files (format: see "stage guides" below). Stage clips are rendered
outside the repo; pass --video NAME=PATH (or --stage1-video PATH) to publish one.
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
S1_VIDEO = os.environ.get('TAILWHIP_STAGE1_VIDEO')      # the stage 1 clip; without it the published copy in dist/ is kept
FFMPEG = '/opt/homebrew/bin/ffmpeg'                     # explicit: another ffmpeg on PATH is broken on the Mac

PAGES = [  # slug, nav label, title, lede
    ('index', 'Overview', 'Tailwhip, the robot that sits on its tail',
     'A two-wheeled balancing robot built from the hobby parts in most robotics drawers. It rests on its tail, rears up onto its wheels, '
     'balances, looks around, and lands on its tail when you shove it. You train its brain with reinforcement learning '
     'in MuJoCo on your laptop, then run it on the real robot.'),
    # the stage guides (stageN/GUIDE.md) are inserted here at build time
    ('build', 'Build plan', 'Build plan', 'From ordering parts to a robot that rears up on its own: seven milestones, each with a test that tells you it worked.'),
    ('parts', 'Parts', 'Parts and prices', 'Everything to buy for one robot, with what to look for in each listing and what you probably already own.'),
    ('mechanical', 'Mechanical', 'Print and assemble', 'Seven prints and one long bolt pair. Spin the models, download the STLs, and measure your TT motor before you print.'),
    ('electronics', 'Electronics', 'Circuit and wiring', 'One ESP32, two TT motors on an MX1508, two servos, an MPU6050 and a 2S 18650 pack. Schematic, wiring table and a safe bring-up.'),
    ('software', 'Software', 'Software', 'Firmware on the robot, a radio bridge on your laptop, and Python tools for driving, bench tests and running the policy.'),
    ('learning', 'Simulation and RL', 'Simulate and train', 'The MuJoCo model built from the CAD, every sloppy part made sloppy on purpose, what the policy sees and is rewarded for, and how to train your own.'),
    ('course', 'Learning path', 'Learning path', 'One-hour sessions from the simulator to a robot that gets back up. One thing to do, one thing to learn and one check in each.'),
    ('risks', 'Limits', 'Limits and gotchas', 'What to know before you order parts: what the design cannot do, what may break, and how to keep the battery safe.'),
]
REFS = {}   # reference pages of the stage guides: slug -> (stage, entry); outside the previous/next path
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

def contents(body, rail=None):
    """'On this page': a sticky column beside the text on wide screens, a fold above it on narrow ones."""
    if rail is None:
        rail = [('h2', i, re.sub(r'<[^>]+>', '', t)) for i, t in re.findall(r'<h2 id="([^"]+)">(.*?)</h2>', body, re.S)]
    if len(rail) < 2:
        return '', ''
    items = ''.join(f'<li class="{lv}"><a href="#{i}">{html.escape(html.unescape(t))}</a></li>' for lv, i, t in rail)
    deep = ' deep' if any(lv == 'h3' for lv, _, _ in rail) else ''
    return (f'<nav class="onpage{deep}" aria-label="On this page"><div class="label">On this page</div><ol>{items}</ol></nav>',
            f'<details class="toc"><summary>On this page</summary><ol>{items}</ol></details>')

def doc(name, drop_title=True):
    p = ROOT / 'docs' / name
    if not p.exists():
        return ''
    t = p.read_text()
    t = re.sub(r'\A# [^\n]*\n', '', t) if drop_title else t
    return t

def page(slug, body, scripts='', rail=None):
    brz = 'https://buildrobotz.com'
    cur = ' aria-current="page"'
    ref = REFS.get(slug)
    if ref:
        st, r = ref
        guide = next(p for p in PAGES if p[0] == st.slug)
        here, label = st.slug, f'{st.nav} reference · {r["label"]}'
        title = f'{st.nav} reference: {r["label"]}'
        lede = r['opts'].get('lede', f'Detail behind the {st.nav} guide. You do not need this page to finish a step; the guide links here when one needs it.')
        others = ' · '.join(f'<a href="{x["slug"]}.html"{cur if x["slug"] == slug else ""}>{html.escape(x["label"])}</a>' for x in st.refs)
        sub = (f'<nav class="subnav" aria-label="{html.escape(st.nav)} reference"><a href="{st.slug}.html">&larr; {html.escape(guide[2])}</a>'
               f'<span>Reference:</span>{others}</nav>')
        prev, nxt = f'<a href="{st.slug}.html">&larr; Back to the {html.escape(st.nav)} guide</a>', '<span></span>'
    else:
        i = [p[0] for p in PAGES].index(slug); _, label, title, lede = PAGES[i]
        here, sub = slug, ''
        prev = f'<a href="{PAGES[i-1][0]}.html">&larr; {PAGES[i-1][1]}</a>' if i else '<span></span>'
        nxt = f'<a href="{PAGES[i+1][0]}.html">{PAGES[i+1][1]} &rarr;</a>' if i + 1 < len(PAGES) else '<span></span>'
    nav = ''.join(f'<a href="{s}.html"{cur if s == here else ""}>{l}</a>' for s, l, _, _ in PAGES)
    body = sized(body)
    side, fold = contents(body, rail)
    if 'class="viewer"' in body and 'viewer.js' not in scripts:
        scripts += VIEWER_JS
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
<div class="wrap layout"><aside class="rail">{side}</aside><main{' class="refpage"' if ref else ''}>
<header><div class="eyebrow">BuildRobotz project · {html.escape(label)}</div><h1>{html.escape(title)}</h1><p class="lede">{html.escape(lede)}</p></header>
{sub}{fold}{fold_reference(body)}
<div class="pager">{prev}{nxt}</div>
</main></div>
<footer><div class="wrap layout"><div></div><div>Tailwhip is an open robot project from <a href="{brz}/">BuildRobotz</a>. The design was generated with AI and has not yet been built or tested on hardware; performance figures are MuJoCo simulation results. Build at your own risk. Code, CAD and firmware: <a href="{REPO_URL}">{REPO_URL.split("//")[1]}</a> (code MIT, docs and CAD CC BY-SA 4.0).<br>
<a href="{brz}/robots/">More robots to build</a> · <a href="https://ouro.buildrobotz.com/">Ouro, its sister project</a> · <a href="{brz}/">Learn robotics with BuildRobotz</a></div></div></footer>
<script>(function(){{var a=document.querySelector('nav [aria-current]');if(a&&a.parentNode.scrollWidth>a.parentNode.clientWidth)a.parentNode.scrollLeft=a.offsetLeft-a.parentNode.clientWidth/2+a.offsetWidth/2;
function op(){{var t=location.hash&&document.getElementById(decodeURIComponent(location.hash.slice(1)));for(var e=t;e;e=e.parentElement)if(e.tagName==='DETAILS')e.open=true;if(t)t.scrollIntoView();}}
addEventListener('hashchange',op);op();
var ls=[].slice.call(document.querySelectorAll('.onpage a'));if(ls.length&&'IntersectionObserver'in window){{var hs=ls.map(function(l){{return document.getElementById(decodeURIComponent(l.hash.slice(1)))}});
var on=function(){{var k=0;hs.forEach(function(h,i){{if(h&&h.getBoundingClientRect().top<120)k=i}});ls.forEach(function(l,i){{l.classList.toggle('here',i===k)}})}};addEventListener('scroll',on,{{passive:true}});on();}}}})();</script>
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
    IMG_SIZE[f'assets/{name}.jpg'] = im.size
    return f'assets/{name}.jpg'

IMG_SIZE = {}   # url -> (w, h) of every picture copied, so pages reserve its space and #links land where they should

def sized(body):
    def add(m):
        tag, src = m.group(0), m.group(1)
        if ' width=' in tag or src not in IMG_SIZE:
            return tag
        w, h = IMG_SIZE[src]
        return tag[:-1] + f' width="{w}" height="{h}">'
    return re.sub(r'<img [^>]*?src="([^"]+)"[^>]*>', add, body)

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
    guides = {st.slug: st for st in STAGES}
    def cards(pages):
        out = ''
        for s, l, t, d in pages:
            st = guides.get(s)
            refs = ('<small>Reference: ' + ' · '.join(html.escape(r['label']) for r in st.refs) + '</small>') if st and st.refs else ''
            out += f'<a href="{s}.html"><b>{t if st else l}</b><span>{d}</span>{refs}</a>'
        return out
    first = [p for p in PAGES if p[0] in guides]; rest = [p for p in PAGES[1:] if p[0] not in guides]
    return f'''
<section class="videos">{vids}</section>
<dl class="facts">{''.join(f"<div><dt>{k}</dt><dd>{v}</dd></div>" for k, v in facts.items())}</dl>
{status_note()}
<section class="prose">{md(doc('overview.md'))}</section>
<section><div class="prose"><h2 id="build-it-page-by-page">Build it, page by page</h2>
<h4>First: {' then '.join(html.escape(st.nav.lower()) for st in STAGES)}</h4></div><div class="cards" style="margin-top:12px">{cards(first)}</div>
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


# ------------------------------------------------------------------------------------------- stage guides
# Each stageN/GUIDE.md becomes one step-by-step guide page in the main path, plus the reference pages its front matter
# lists (rendered from that stage's own docs, outside the previous/next path). The guide's words are in GUIDE.md;
# every number, file, table, picture and diagram is pulled from the stage's files at build time. Format:
#
#   ---                                      front matter, one 'key: value' per line
#   slug: stage1                             page name (stage1.html); also: nav, title, lede
#   reference: stage1-wiring | Electronics | docs/electronics.md | fold=Sources;Firmware notes | lede=...
#   reference: stage1-sim | Simulation | README.md#What the simulation says + out/sim.md
#   fact: cost = README.md: \*\*about (\$\d+)\*\* || other.md: ...    first capture group; '$a.b' reads a JSON path
#   ---
#   Top block (markdown). {{cost}} inserts a fact; a line whose fact is missing is left out, with a warning.
#   # Milestone A: ...                       a milestone; its first paragraph is the milestone's introduction
#   ## Buy the parts                         a step, numbered automatically across milestones
#   ![caption](out/step_*core*.png | out/render_parts.png)   the step's picture: first existing match, else skipped
#   **Check:** ...                           the step's check;  **If it does not work:** ... its pointer to the reference
#   [[stl out/print/*.stl]]  [[svg docs/wiring.svg | caption]]  [[table README.md | Parts]]  [[para README.md | text]]
#   [[status README.md | title]]  [[facts Label=value | ...]]  [[video name | caption]]  [[file path | label]]  [[viewer globs]]
STAGES = []
WARNINGS = []
VIEWER_JS = '''<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/GLTFLoader.js"></script>
<script src="assets/viewer.js"></script>'''
SITE_DOCS = {'docs/electronics.md': 'electronics.html', 'docs/mechanical.md': 'mechanical.html', 'docs/build.md': 'build.html',
             'docs/parts.md': 'parts.html', 'docs/software.md': 'software.html', 'docs/limits.md': 'risks.html'}
DOC_PAGE = {'overview': 'index.html', 'build': 'build.html', 'parts': 'parts.html', 'mechanical': 'mechanical.html',
            'electronics': 'electronics.html', 'software': 'software.html', 'simulation': 'learning.html',
            'learning-path': 'course.html', 'limits': 'risks.html'}
COLORS = ('#e8862a', '#e8862a', '#f2b84b', '#f2b84b', '#2a9ea8', '#cfcfc8', '#e9e4d8')

def warn(msg):
    WARNINGS.append(msg)
    print('  warning:', msg)

def slugify(t):
    return re.sub(r'[^a-z0-9]+', '-', re.sub(r'<[^>]+>', '', t).lower()).strip('-')

def sections(text):
    """(preamble, [(title, body)]) split at '## ' headings, with the '# ' title dropped."""
    text = re.sub(r'\A# [^\n]*\n', '', text)
    parts = re.split(r'^## (.+)\n', text, flags=re.M)
    return parts[0], [(parts[i].strip(), parts[i + 1]) for i in range(1, len(parts), 2)]

def joined(secs):
    return ''.join(f'## {t}\n{b}' for t, b in secs)

def starts(t, names):
    return any(t.lower().startswith(n.strip().lower()) for n in names if n.strip())

def folded(secs, names):
    """Sections named in names go last, under '## Reference', each folding into a <details> on the page."""
    keep = [(t, b) for t, b in secs if not starts(t, names)]
    fold = [(t, b) for t, b in secs if starts(t, names)]
    out = joined(keep)
    if fold:
        out += '## Reference\n\n' + ''.join(f'### {t}\n\n' + re.sub(r'^(#{3,5}) ', r'#\1 ', b, flags=re.M) + '\n' for t, b in fold)
    return out

def btns(items):
    return ' '.join(f'<a class="btn" href="{u}"{(" download=" + chr(34) + d + chr(34)) if isinstance(d, str) else " download" if d else ""}>{html.escape(l)}</a>'
                    for l, u, d in items if u)

def encode_video(src, name):
    """A clip re-encoded for the web into assets/video/<name>.mp4 with a poster; without a source the published copy stays."""
    src = pathlib.Path(src).expanduser()
    if not src.exists():
        raise SystemExit(f'video {name}: {src} not found')
    (DIST / 'assets' / 'video').mkdir(parents=True, exist_ok=True)
    subprocess.run([FFMPEG, '-y', '-loglevel', 'error', '-i', str(src), '-vf', 'scale=854:-2', '-r', '30', '-c:v', 'h264_videotoolbox',
                    '-b:v', '700k', '-pix_fmt', 'yuv420p', '-an', '-movflags', '+faststart', str(DIST / f'assets/video/{name}.mp4')], check=True)
    subprocess.run([FFMPEG, '-y', '-loglevel', 'error', '-ss', '1', '-i', str(src), '-frames:v', '1', '-vf', 'scale=960:-2',
                    '-q:v', '4', str(DIST / f'assets/video/{name}_poster.jpg')], check=True)


class Stage:
    def __init__(self, guide):
        self.dir = guide.parent; self.name = self.dir.name
        text = guide.read_text()
        m = re.match(r'---\n(.*?)\n---\n', text, re.S)
        if not m:
            raise SystemExit(f'{guide}: no front matter')
        self.meta, self.refs, self.facts = {}, [], {}
        for line in m.group(1).splitlines():
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            k, _, v = line.partition(':'); k, v = k.strip(), v.strip()
            if k == 'reference':
                p = [x.strip() for x in v.split(' | ')]
                self.refs.append(dict(slug=p[0], label=p[1], src=p[2], opts=dict(x.split('=', 1) for x in p[3:])))
            elif k == 'fact':
                name, _, spec = v.partition(' = ')
                self.facts[name.strip()] = self.fact(spec.strip(), name.strip())
            else:
                self.meta[k] = v
        self.body = text[m.end():]
        self.slug = self.meta['slug']; self.nav = self.meta.get('nav', self.name)
        self.status = ''
        # where links to this stage's files go on the site
        self.site = {f'{self.name}/README.md': f'{self.slug}.html'}
        for r in self.refs:
            for spec in r['src'].split(' + '):
                f = spec.partition('#')[0]
                if '#' not in spec:
                    self.site.setdefault(f'{self.name}/{f}', f'{r["slug"]}.html')

    def fact(self, spec, name):
        for alt in spec.split(' || '):
            f, _, rx = alt.partition(': ')
            p = self.dir / f.strip()
            if not p.exists():
                continue
            if rx.startswith('$'):
                v = json.loads(p.read_text())
                try:
                    for k in rx[1:].split('.'):
                        v = v[int(k)] if isinstance(v, list) else v[k]
                    return str(v)
                except (KeyError, IndexError, ValueError, TypeError):
                    continue
            m = re.search(rx, p.read_text(), re.S)
            if m:
                return next((g for g in m.groups() if g is not None), m.group(0)) if m.groups() else m.group(0)
        warn(f'{self.name}: fact "{name}" not found in the stage files; lines that use it are left out')
        return None

    def read(self, rel):
        p = self.dir / rel
        return p.read_text() if p.exists() else None

    def files(self, globs):
        out, skip = [], {h for g in globs.split() if g.startswith('!') for h in self.dir.glob(g[1:])}
        for g in globs.split():
            if g.startswith('!'):
                continue
            hits = [h for h in sorted(self.dir.glob(g)) if h not in skip]
            if not hits:
                warn(f'{self.name}: nothing matches {g}; skipped')
            out += [h for h in hits if h not in out]
        return out

    def first(self, alts):
        for g in alts.split(' | '):
            hits = sorted(self.dir.glob(g.strip()))
            if hits:
                return hits[0]
        warn(f'{self.name}: no picture for {alts!r}; skipped')
        return None

    # -------------------------------------------------- links and pictures
    def url(self, p):
        """Where the site points for a repo file: a page or asset if there is one, else the file on GitHub."""
        p = p.resolve()
        try:
            rel = p.relative_to(ROOT).as_posix()
        except ValueError:
            return None
        if rel in self.site:
            return self.site[rel]
        if rel in SITE_DOCS:
            return SITE_DOCS[rel]
        if rel.startswith(f'{self.name}/out/') and p.suffix == '.png' and p.exists():
            return self.img(p)
        if p.exists():
            return f'{REPO_URL}/{"tree" if p.is_dir() else "blob"}/main/{rel}'
        return None

    def find(self, path, base):
        """A path as the stage docs write them: next to the doc, then from the stage folder, then from the repo root
        ('docs/...' inside the stage's docs/ means the full robot's docs)."""
        cands = [self.dir / base / path]
        if not (base and path.startswith('docs/')):
            cands.append(self.dir / path)
        cands.append(ROOT / path)
        return next((c for c in cands if c.exists()), None)

    def img(self, p):
        return copy_img(p, f'{self.name}_{p.stem}')

    def svg(self, p, cap):
        s = p.read_text()
        m = re.search(r"<svg[^>]*?width=['\"](\d+)['\"][^>]*?height=['\"](\d+)['\"]", s)
        size = f' width="{m.group(1)}" height="{m.group(2)}"' if m else ''
        url = copy_file(p, f'assets/{self.name}_{p.name}')
        return (f'<figure class="fig wide"><div class="svgwrap native"><a href="{url}"><img src="{url}" alt="{html.escape(cap)}"{size}></a></div>'
                f'<figcaption><b>{html.escape(cap.upper())}</b> · drawn full size: scroll inside the frame, or '
                f'<a href="{url}" download="tailwhip_{self.name}_{p.name}">download {p.name}</a></figcaption></figure>')

    def md(self, text, base, tokens, seen=None):
        """Stage markdown ready for md(): pictures copied (SVG diagrams full size), links and `paths` pointed at the site."""
        def image(m):
            alt, path = m.group(1), m.group(2)
            p = (self.dir / base / path).resolve()
            if seen is not None:
                if p in seen:
                    return ''
                seen.add(p)
            if not p.exists():
                warn(f'{self.name}: picture {path} missing; skipped'); return ''
            if p.suffix == '.svg':
                return token(tokens, self.svg(p, alt or p.stem))
            url = self.img(p) if p.suffix == '.png' else copy_file(p, f'assets/{self.name}_{p.name}')
            return f'![{alt}]({url})'
        text = re.sub(r'!\[([^\]]*)\]\(([^)\s]+)\)', image, text)
        def link(m):
            label, path = m.group(1), m.group(2)
            if re.match(r'[a-z]+:|#', path) or path.endswith('.html') or '.html#' in path:
                return m.group(0)
            f, _, frag = path.partition('#')
            url = self.url(self.dir / base / f)
            return f'[{label}]({url}{"#" + frag if frag else ""})' if url else m.group(0)
        text = re.sub(r'(?<![!\]])\[([^\]]+)\]\(([^)\s]+)\)', link, text)
        def code(m):
            path = m.group(1)
            if not re.search(r'[./]', path):
                return m.group(0)
            found = self.find(path, base)
            url = self.url(found) if found else None
            return f'[`{path}`]({url})' if url else m.group(0)
        return re.sub(r'(?<![`\[])`([\w.][\w./-]*)`(?![`\]])', code, text)

    # -------------------------------------------------- pieces pulled from the stage files
    def block(self, rel, needle):
        """The paragraph or list item in a stage file that contains needle."""
        t = self.read(rel)
        if t is None:
            return None
        for b in re.split(r'\n\s*\n|\n(?=\s*(?:\d+\.|[-*]) )', t):
            if needle in b:
                return re.sub(r'^\s*(?:\d+\.|[-*])\s+', '', b.strip())
        return None

    def table(self, rel, heading):
        """The first markdown table under the heading (any level) that starts with heading."""
        t = self.read(rel)
        if t is None:
            return None
        m = re.search(r'^#{1,5} ' + re.escape(heading) + r'.*?\n(.*?)(?=^#{1,5} |\Z)', t, re.S | re.M)
        tab = m and re.search(r'((?:^\|.*\|[ \t]*\n?)+)', m.group(1), re.M)
        return tab.group(1) if tab else None


def token(tokens, htm):
    tokens.append(htm)
    return f'\n\nXTOKEN{len(tokens) - 1}X\n\n'

def untoken(out, tokens):
    for i, h in enumerate(tokens):
        out = out.replace(f'<p>XTOKEN{i}X</p>', h).replace(f'XTOKEN{i}X', h)
    return out

def directive(st, kind, args, tokens):
    a = [x.strip() for x in args.split(' | ')] if args else []
    base = ''
    if kind == 'stl':
        fs = st.files(a[0]) if a else []
        items = []
        for f in fs:
            dst = copy_file(f, f'models/{st.name}/{f.name}')
            items.append((f.name, dst, f'tailwhip_{st.name}_{f.name}'))
        return f'<p class="files">{btns(items)}</p>' if items else ''
    if kind == 'viewer':
        fs = [f for f in st.files(a[0]) if f.suffix in ('.stl', '.glb')] if a else []
        models = [dict(label=f.stem.replace('_', ' '), file=copy_file(f, f'models/{st.name}/{f.name}'), color=COLORS[i % len(COLORS)])
                  for i, f in enumerate(fs)]
        if not models:
            return ''
        return (f'<section class="wide"><div class="viewer" data-models=\'{json.dumps(models)}\'><canvas class="view"></canvas>'
                '<span class="hint">drag to orbit · scroll to zoom</span></div>'
                '<div class="bar vbar" style="border:1px solid var(--rule);border-top:0;border-radius:0 0 6px 6px"></div></section>')
    if kind == 'svg':
        p = st.dir / a[0]
        if not p.exists():
            warn(f'{st.name}: diagram {a[0]} missing; skipped'); return ''
        return st.svg(p, a[1] if len(a) > 1 else p.stem)
    if kind == 'table':
        t = st.table(a[0], a[1])
        if not t:
            warn(f'{st.name}: no table under "{a[1]}" in {a[0]}; skipped'); return ''
        return md(st.md(t, str(pathlib.Path(a[0]).parent).replace('.', ''), tokens))
    if kind == 'para':
        b = st.block(a[0], a[1])
        if not b:
            warn(f'{st.name}: no paragraph with "{a[1]}" in {a[0]}; skipped'); return ''
        htm = md(st.md(b, str(pathlib.Path(a[0]).parent).replace('.', ''), tokens))
        return f'<div class="check"><b>{html.escape(a[2])}</b>{htm}</div>' if len(a) > 2 else htm
    if kind == 'status':
        b = st.block(a[0], '**Status:**')
        if not b:
            warn(f'{st.name}: no **Status:** line in {a[0]}'); return ''
        b = re.sub(r'\s+', ' ', b.replace('**Status:**', '')).strip(); b = b[0].upper() + b[1:]
        st.status = ('<div class="note"><b>' + html.escape(a[1] if len(a) > 1 else 'Status') + '</b>'
                     + re.sub(r'^<p>|</p>$', '', markdown.markdown(b)) +
                     ' Like the rest of Tailwhip, the design, its code and these pages were generated with AI. Measure what the pages tell you to, '
                     'check each step with a meter, expect to fix things, and build at your own risk, especially around the 18650 cells.</div>')
        return st.status
    if kind == 'facts':
        cells = [x.partition('=') for x in a]
        return '<dl class="facts">' + ''.join(f'<div><dt>{html.escape(k.strip())}</dt><dd>{html.escape(v.strip())}</dd></div>' for k, _, v in cells) + '</dl>'
    if kind == 'video':
        v, poster = published(f'assets/video/{a[0]}.mp4'), published(f'assets/video/{a[0]}_poster.jpg')
        if not v:
            warn(f'{st.name}: video {a[0]} not published; skipped (build once with --video {a[0]}=PATH)'); return ''
        p = f' poster="{poster}"' if poster else ''
        return (f'<figure class="fig"><video src="{v}"{p} controls muted loop playsinline preload="metadata"></video>'
                f'<figcaption>{html.escape(a[1]) if len(a) > 1 else ""}</figcaption></figure>')
    if kind == 'file':
        p = st.dir / a[0]
        if not p.exists():
            warn(f'{st.name}: file {a[0]} missing; skipped'); return ''
        rel = p.resolve().relative_to(ROOT).as_posix()
        return f'<p class="files">{btns([(a[1] if len(a) > 1 else p.name, f"{REPO_URL}/blob/main/{rel}", False), ("Download", f"{REPO_URL}/raw/main/{rel}", False)])}</p>'
    raise SystemExit(f'{st.name}: unknown directive [[{kind}]]')

def fill_facts(st, text):
    out = []
    for line in text.split('\n'):
        names = re.findall(r'\{\{(\w+)\}\}', line)
        if any(st.facts.get(n) is None for n in names):
            continue                                    # the warning was given when the fact was read
        out.append(re.sub(r'\{\{(\w+)\}\}', lambda m: st.facts[m.group(1)], line))
    return '\n'.join(out)

def render_md(st, text, tokens, picture=False):
    """Directives and pictures to tokens, then markdown. With picture=True the first picture is returned separately."""
    lead = None
    def pic(m):
        nonlocal lead
        cap, alts = m.group(1), m.group(2)
        p = st.first(alts)
        if not p:
            return ''
        htm = st.svg(p, cap) if p.suffix == '.svg' else fig(st.img(p), html.escape(cap))
        if picture and lead is None:
            lead = htm; return ''
        return token(tokens, htm)
    text = re.sub(r'^!\[([^\]]*)\]\(([^)]+)\)[ \t]*$', pic, text, flags=re.M)
    def direc(m):
        nonlocal lead
        htm = directive(st, m.group(1), m.group(2).strip(), tokens)
        if picture and lead is None and m.group(1) == 'video' and htm:
            lead = htm; return ''
        return token(tokens, htm)
    text = re.sub(r'^\[\[(\w+)\s*(.*?)\]\][ \t]*$', direc, text, flags=re.M)
    return untoken(md(st.md(text, '', tokens)), tokens), lead

def render_guide(st):
    tokens = []
    body = fill_facts(st, st.body)
    parts = re.split(r'^(#{1,2}) (.+)\n', body, flags=re.M)
    top, _ = render_md(st, parts[0], tokens)
    out, rail, n = [f'<section class="prose guide-top">{top}</section>'], [], 0
    words = len(re.sub(r'<[^>]+>', ' ', top).split())
    for i in range(1, len(parts), 3):
        level, title, text = parts[i], parts[i + 1].strip(), parts[i + 2]
        if level == '#':
            hid = slugify(title.split(':')[0])
            htm, _ = render_md(st, text, tokens)
            out.append(f'<section class="prose milestone"><h2 id="{hid}">{html.escape(title)}</h2>{htm}</section>')
            rail.append(('h2', hid, title))
            words += len(re.sub(r'<[^>]+>', ' ', htm).split()) + len(title.split())
            continue
        n += 1; hid = f'step-{n}'
        chk = re.search(r'^\*\*Check:\*\*\s*(.+?)(?=\n\s*\n|\n\*\*If it does not work|\Z)', text, re.S | re.M)
        ifn = re.search(r'^\*\*If it does not work:\*\*\s*(.+?)(?=\n\s*\n|\Z)', text, re.S | re.M)
        cap = lambda x: x.strip()[:1].upper() + x.strip()[1:]
        for m in (chk, ifn):
            if m:
                text = text.replace(m.group(0), '')
        wide = []
        def widen(m):
            wide.append(m.group(0)); return ''
        text = re.sub(r'^\[\[(?:svg|viewer)\b.*?\]\][ \t]*$', widen, text, flags=re.M)
        htm, lead = render_md(st, text, tokens, picture=True)
        wide_htm = ''.join(render_md(st, w, tokens)[0] for w in wide)
        tail = ''
        if chk:
            tail += f'<div class="check"><b>Check</b>{re.sub(r"^<p>|</p>$", "", render_md(st, cap(chk.group(1)), tokens)[0].strip())}</div>'
        if ifn:
            tail += f'<p class="ifnot"><b>If it does not work:</b> {re.sub(r"^<p>|</p>$", "", render_md(st, ifn.group(1), tokens)[0].strip())}</p>'
        words += sum(len(re.sub(r'<[^>]+>', ' ', x).split()) for x in (htm, tail)) + len(title.split())
        out.append(f'<section class="step" id="{hid}"><h3><span class="num">{n}</span>{html.escape(title)}</h3>'
                   f'<div class="step-main{" has-lead" if lead else ""}">{lead or ""}<div class="prose">{htm}</div></div>{wide_htm}{tail}</section>')
        rail.append(('h3', hid, f'{n}. {title}'))
    st.words = words
    print(f'  {st.slug}: {n} steps, about {words} words (tables and captions included)')
    return '\n'.join(out), rail

def render_ref(st, ref):
    tokens, seen, out = [], set(), ''
    opts = ref['opts']
    for spec in ref['src'].split(' + '):
        f, _, names = spec.strip().partition('#')
        text = st.read(f)
        if text is None:
            warn(f'{st.name}: {f} missing; left out of {ref["slug"]}'); continue
        base = str(pathlib.Path(f).parent).replace('.', '')
        pre, secs = sections(text)
        if names:
            want = names.split(';')
            secs = [(t, b) for t, b in secs if starts(t, want)]
            if not secs:
                warn(f'{st.name}: none of {names!r} in {f}')
            pre = ''
        text = pre + folded(secs, opts.get('fold', '').split(';'))
        out += f'<section class="prose">{untoken(md(st.md(text, base, tokens, seen)), tokens)}</section>'
    if opts.get('viewer'):
        out = directive(st, 'viewer', opts['viewer'], tokens) + directive(st, 'stl', opts['viewer'], tokens) + out
    note = (f'<div class="note refnote"><b>Reference</b>This page is detail behind the <a href="{st.slug}.html">{html.escape(st.nav)} guide</a>. '
            'You can build without reading it: each step of the guide links to the section it needs when something goes wrong. '
            'Like everything in Tailwhip, it is designed and simulated, not built.</div>')
    return note + out

def load_stages():
    for g in sorted(ROOT.glob('stage*/GUIDE.md')):
        st = Stage(g)
        STAGES.append(st)
        PAGES.insert(1 + len(STAGES) - 1, (st.slug, st.nav, st.meta.get('title', st.nav), st.meta.get('lede', '')))
        for r in st.refs:
            REFS[r['slug']] = (st, r)

def check_links():
    pages = {p.name: p.read_text() for p in DIST.glob('*.html')}
    ids = {k: set(re.findall(r'\sid="([^"]+)"', v)) for k, v in pages.items()}
    for name, t in pages.items():
        for u in re.findall(r'(?:href|src)="([^"#:]*)(?:#([^"]*))?"', t):
            f, frag = u
            if f.startswith(('http', 'mailto', 'data')):
                continue
            f = f or name
            if not (DIST / f).exists():
                warn(f'{name}: broken link {f}')
            elif frag and f.endswith('.html') and frag not in ids.get(f, ()):
                warn(f'{name}: no #{frag} in {f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--anim', help='path to the Blender animation mp4 (or set TAILWHIP_ANIM); without it the published copy in dist/ is kept')
    ap.add_argument('--video', action='append', default=[], metavar='NAME=PATH',
                    help='a clip for a stage guide\'s [[video NAME]], re-encoded for the web with ' + FFMPEG + '; without it the published copy is kept')
    ap.add_argument('--stage1-video', help='same as --video stage1_sim=PATH (the nominal run of stage1/sim.py)')
    args = ap.parse_args()
    global ANIM
    if args.anim:
        ANIM = pathlib.Path(args.anim).expanduser()
    videos = dict(v.split('=', 1) for v in args.video)
    if args.stage1_video or S1_VIDEO:
        videos['stage1_sim'] = args.stage1_video or S1_VIDEO
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
    for name, src in videos.items():
        encode_video(src, name)
    load_stages()
    guides = {st.slug: st for st in STAGES}
    for slug, *_ in PAGES:
        rail = None
        if slug in guides:
            body, rail = render_guide(guides[slug]); scripts = ''
        else:
            r = globals()['p_' + slug.replace('-', '_')]()
            body, scripts = r if isinstance(r, tuple) else (r, '')
        body = re.sub(r'(src|href)="(?:\./)?(schematic\.svg|wiring\.svg|perfboard\.svg|electronics\.json)"', r'\1="assets/\2"', body)
        body = re.sub(r'(src|href)="\.\./cad/out/([\w-]+)\.png"', r'\1="assets/cad_\2.jpg"', body)
        body = re.sub(r'href="(?:\./)?([\w-]+)\.md(#[^"]*)?"', lambda m: f'href="{DOC_PAGE[m.group(1)]}{m.group(2) or ""}"'
                      if m.group(1) in DOC_PAGE else m.group(0), body)     # docs link to each other as .md files
        (DIST / f'{slug}.html').write_text(page(slug, body, scripts, rail))
    for slug, (st, ref) in REFS.items():
        (DIST / f'{slug}.html').write_text(page(slug, render_ref(st, ref)))
    # images referenced from the docs as ../cad/out/*.png
    for m in set(re.findall(r'assets/cad_([\w-]+)\.jpg', ''.join(p.read_text() for p in DIST.glob('*.html')))):
        if not (DIST / 'assets' / f'cad_{m}.jpg').exists():
            copy_img(ROOT / 'cad' / 'out' / f'{m}.png', f'cad_{m}')
    (DIST / 'CNAME').write_text(DOMAIN + '\n')
    (DIST / '.nojekyll').write_text('')
    check_links()
    n = sum(1 for _ in DIST.rglob('*') if _.is_file()); size = sum(f.stat().st_size for f in DIST.rglob('*') if f.is_file())
    print(f'built {len(PAGES)} pages + {len(REFS)} reference pages, {n} files, {size / 1e6:.1f} MB -> {DIST}'
          + (f'; {len(WARNINGS)} warnings' if WARNINGS else ''))

if __name__ == '__main__':
    main()
