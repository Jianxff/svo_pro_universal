import * as THREE from 'https://threejsfundamentals.org/threejs/resources/threejs/r132/build/three.module.js'
import { OrbitControls } from 'https://threejsfundamentals.org/threejs/resources/threejs/r132/examples/jsm/controls/OrbitControls.js';
// import { TransformControls } from 'three/addons/controls/TransformControls.js';

class Spacial {
  // basic scene settings
  fov = 60
  aspect = 4 / 3
  alpha = true
  near = 0.1
  far = 1000
  animation = false
  on_frame = ()=>{}
  ambient = 1

  // scene objects
  scene_ = null;
  renderer_ = null;
  cameras_ = [];
  orbits_ = {};
  
  // control
  width_ = 0;
  height_ = 0;
  view_id_ = null;
  animation_id_ = null;

  constructor(options) {
    // set options
    for (let key in options) {
      if(this[key] == undefined) 
        throw new Error(`[Scene] invalid option: ${key}`);
      this[key] = options[key];
    }
    // init objects
    this.scene_ = new THREE.Scene()
    this.renderer_ = new THREE.WebGLRenderer({ alpha: this.alpha, antialias: true })
    this.renderer_.setPixelRatio(window.devicePixelRatio)
    // light
    const light = new THREE.AmbientLight(0xffffff, this.ambient)
    this.scene_.add(light)
  }

  // add camera
  addCamera(type = 'perspective', options = {}) {
    // check options
    var camera;
    if(type == 'perspective') {
      if(!options['fov']) options.fov = this.fov;
      if(!options['aspect']) options.aspect = this.aspect;
      if(!options['near']) options.near = this.near;
      if(!options['far']) options.far = this.far;
      camera = new THREE.PerspectiveCamera(options.fov, options.aspect, options.near, options.far)
    } else if(type == 'orthographic') {
      if(!options['left']) options.left = -1;
      if(!options['right']) options.right = 1;
      if(!options['top']) options.top = 1;
      if(!options['bottom']) options.bottom = -1;
      camera = new THREE.OrthographicCamera(options.left, options.right, options.top, options.bottom, options.near, options.far)
    } else throw new Error(`[Scene] invalid camera type: ${type}`);
    // add to scene
    this.cameras_.push(camera)
    this.scene_.add(camera)
    if(this.cameras_.length == 1) this.view_id_ = 0;
    
    return (this.cameras_.length - 1)
  }

  // orbit
  addOrbitControl(camera_id = null, max_dist = null, min_dist = null) {
    camera_id = (camera_id == null ? this.view_id_ : camera_id);
    if(camera_id < 0 || camera_id >= this.cameras_.length) 
      throw new Error(`[Scene] invalid camera_id: ${camera_id}`);

    const controls = new OrbitControls(this.cameras_[camera_id], this.renderer_.domElement)
    // set min / max
    controls.maxDistance = (max_dist == null ? this.far : max_dist)
    controls.minDistance = (min_dist == null ? this.near : min_dist)
    // set event listener
    controls.addEventListener('change', ()=>{this.render()})
    // add to scene
    this.orbits_[camera_id] = controls;
    return this;
  }

  setOrbits(val, idx = null) {
    if(idx != null && !Array.isArray(idx)) idx = [idx];
    if(idx == null) idx = Object.keys(this.orbits_);
    for(let key in this.orbits_) {
      this.orbits_[key].enabled = val;
    }
    return this;
  }
    
  // start render
  bind(domElement) {
    domElement.appendChild(this.renderer_.domElement)
    return this;
  }

  // render
  render() {
    this.renderer_.render(this.scene_, this.cameras_[this.view_id_])
  }

  // set size
  resetSize(w, h) {
    this.width_ = w
    this.height_ = h
    this.renderer_.setSize(w, h)
    console.log('resize to ' + w + 'x' + h)
    return this;
  }

  // on frame
  setOnFrame(hook) {
    this.stop();
    this.on_frame = hook;
    return this;
  }

  // start render
  animation_loop() {
    this.on_frame(this);
    this.render();
    this.animation_id_ = requestAnimationFrame(() => this.animation_loop());
  }
  start() {
    if(this.animation && this.animation_id_)
      cancelAnimationFrame(this.animation_id_)

    if(this.animation)
      this.animation_id_ = this.animation_loop()
    
    return this;
  }

  // stop animation
  stop() {
    if(this.animation && this.animation_id_)
      cancelAnimationFrame(this.animation_id_)
  }

  // switch view
  switchView(camera_id) {
    if(camera_id < 0 || camera_id >= this.cameras_.length) 
      throw new Error(`[Scene] invalid camera_id: ${camera_id}`);
    this.view_id_ = camera_id
    if(this.orbits_[camera_id]) this.orbits_[camera_id].update();
    this.render();
    return this;
  }

  // add objects
  addObjects(obj_list, vis = true) {
    if(!Array.isArray(obj_list)) obj_list = [obj_list];
    // add to scene
    obj_list.forEach(obj => {
      if(obj) this.scene_.add(obj)
      if(!vis) obj.material.visible = false
    })
    this.render();
    return this;
  }

  // remove object
  removeObjects(obj_list) {
    if(!Array.isArray(obj_list)) obj_list = [obj_list];
    // remove from scene
    obj_list.forEach(obj => {
      if(obj) this.scene_.remove(obj)
    })
    this.render();
    return this;
  }

  // set vis
  setObjectsVisible(obj_list, val) {
    if(!Array.isArray(obj_list)) obj_list = [obj_list];
    // set visiblity
    obj_list.forEach(obj => {
      if(obj) obj.material.visible = val
    })
    this.render();
    return this;
  }

  // update camera
  updateCamera(hook = ()=>{}, index = null) {
    index = (index == null ? this.view_id_ : index);
    hook(this.cameras_[index])
    if(this.orbits_[index]) this.orbits_[index].update();
    this.render();
    return this;
  }

  // fit camera
  fitProjection(camera_indexs = null) {
    if(camera_indexs == null) {
      camera_indexs = this.cameras_.map((_, i) => i)
    }
    camera_indexs.forEach(index => {
      const cam = this.cameras_[index];
      if(cam && cam.type == 'PerspectiveCamera') {
        this.cameras_[index].aspect = this.width_ / this.height_;
        this.cameras_[index].updateProjectionMatrix();
      }
    })
    return this;
  }

  // transform
  // createTransformControl(onMouseDown=()=>{}, onMouseUp=()=>{}){
  //   const control = new TransformControls(this.cameras_[this.view_id_], this.renderer_.domElement)
  //   control.setMode('translate')
  //   control.addEventListener('change', ()=>{this.render()})
  //   control.addEventListener('mouseDown', ()=>{
  //     this.setOrbits(false);
  //     onMouseDown();
  //   })
  //   control.addEventListener('mouseUp', ()=>{
  //     this.setOrbits(true);
  //     onMouseUp();
  //   })
  //   this.scene_.add(control)
  //   this.render();
  //   return control;
  // }

  // get scene
  getScene() {
    return this.scene_;
  }

  // get camera
  getCamera(index = null) {
    index = (index == null ? this.view_id_ : index);
    return this.cameras_[index];
  }

  // get main camera id
  getViewIndex() {
    return this.view_id_;
  }

  // get orbit
  getOrbit(index = null) {
    index = (index == null ? this.view_id_ : index);
    return this.orbits_[index];
  }

  // transform
  static formatTwcArray(raw) {
    var array = []
    for(let i = 0; i < 4; i++)
      for(let j = 0; j < 4; j++)
        array.push(raw[i + j * 4]) // col-order !!!!!!!!!!!
    
    const T = new THREE.Matrix4();
    T.fromArray(array);
    const R = new THREE.Matrix4().extractRotation(T);
    const t = new THREE.Vector3(array[12], array[13], array[14]);
    
    return {
      T: T, R:R, t: t
    }
  }
}

export { THREE, Spacial };