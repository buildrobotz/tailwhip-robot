// 3D part viewer. Each .viewer has data-models='[{"label":..,"file":..,"color":..}]'. Needs three.js + STLLoader + GLTFLoader + OrbitControls.
(function(){
  if (!window.THREE) return;
  document.querySelectorAll('.viewer').forEach(box => {
    const models = JSON.parse(box.dataset.models), view = box.querySelector('.view'), bar = box.parentElement.querySelector('.vbar');
    const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
    const renderer = new THREE.WebGLRenderer({canvas: view, antialias: true, alpha: true});
    const scene = new THREE.Scene(), cam = new THREE.PerspectiveCamera(35, 16 / 9, 1, 5000);
    scene.add(new THREE.HemisphereLight(0xffffff, 0x556655, 0.95));
    const key = new THREE.DirectionalLight(0xffffff, 0.75); key.position.set(120, -200, 260); scene.add(key);
    const ctl = new THREE.OrbitControls(cam, view); ctl.enableDamping = true; cam.up.set(0, 0, 1);
    let cur = null;
    function fit(obj){
      const b = new THREE.Box3().setFromObject(obj), c = b.getCenter(new THREE.Vector3()), s = b.getSize(new THREE.Vector3()).length();
      ctl.target.copy(c); cam.position.set(c.x + s * 0.85, c.y - s * 1.25, c.z + s * 0.7); cam.near = s / 100; cam.far = s * 20; cam.updateProjectionMatrix();
    }
    function show(m, btn){
      if (cur) scene.remove(cur);
      bar.querySelectorAll('button').forEach(b => b.setAttribute('aria-pressed', b === btn));
      const done = obj => { cur = obj; scene.add(obj); fit(obj); };
      if (/\.glb$/i.test(m.file)) new THREE.GLTFLoader().load(m.file, g => { g.scene.rotation.x = Math.PI / 2; done(g.scene); });   // glTF is Y-up, the robot is Z-up
      else new THREE.STLLoader().load(m.file, geo => { geo.computeVertexNormals();
        done(new THREE.Mesh(geo, new THREE.MeshStandardMaterial({color: m.color || css('--link-a') || '#d9560f', roughness: 0.75, metalness: 0.05}))); });
    }
    models.forEach((m, n) => { const b = document.createElement('button'); b.type = 'button'; b.textContent = m.label; b.onclick = () => show(m, b); bar.appendChild(b); if (n === 0) show(m, b); });
    function size(){ const w = view.clientWidth, h = view.clientHeight; if (view.width !== w * devicePixelRatio) { renderer.setPixelRatio(devicePixelRatio); renderer.setSize(w, h, false); cam.aspect = w / h; cam.updateProjectionMatrix(); } }
    (function loop(){ size(); ctl.update(); renderer.render(scene, cam); requestAnimationFrame(loop); })();
  });
})();
