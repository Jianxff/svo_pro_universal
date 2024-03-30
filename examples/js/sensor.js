class Sensor{
    _requested = false;
    // motion
    accelerate = null;
    gyroscope = null;
    // media
    stream = null;
    track = null;
    imageCapture = null;
    video_el = null;
    context = null;

    getMotion(arrayLike=false) {
        return {
            accelerate: arrayLike ? [this.accelerate.x, this.accelerate.y, this.accelerate.z] : this.accelerate,
            gyroscope: arrayLike ? [this.gyroscope.alpha, this.gyroscope.beta, this.gyroscope.gamma] : this.gyroscope,
            timestamp: Date.now()
        }
    }

    async getFrame() {
        try{
            var bitmap;
            if(self.imageCapture) {
                bitmap = await this.imageCapture.grabFrame();
            } else {
                bitmap = this.video_el;
            }
            if(!this.context) {
                this.context = new OffscreenCanvas(bitmap.width, bitmap.height).getContext('2d', { willReadFrequently: true });
            }
            context.drawImage(bitmap, 0, 0);
            return {
                data: context.getImageData(0, 0, bitmap.width, bitmap.height).data,
                timestamp: Date.now()
            }
        }catch(err){
            // ignore
        }
        return null;
    }

    async getFrameBitmap() {
        var bitmap;
        if(self.imageCapture) {
            bitmap = await this.imageCapture.grabFrame();
        } else {
            bitmap = this.video_el;
        }
        return {
            data: bitmap,
            timestamp: Date.now()
        }
    }

    getTrack() {
        return this.track;
    }

    getStream() {
        return this.stream;
    }

}

const DEFAULT_CONSTRAINT = {
    video: {
        facingMode: 'environment',
        width: 640,
        height: 480
    },
    audio: false
}

const __SENSOR__ = new Sensor();

async function requestMotion() {
    return new Promise((resolve, reject) => {
        const assert = () => {
            if(DeviceMotionEvent === undefined) {
                reject(new Error("DeviceMotion is not supported."));
                return;
            }
            if(DeviceOrientationEvent === undefined) {
                reject(new Error("DeviceOrientation is not supported."));
                return;
            }
            resolve();
        }

        if(DeviceMotionEvent.requestPermission !== undefined) {
            DeviceMotionEvent.requestPermission()
            .then(state => {
                if(state === "granted") assert();
                else reject(new Error("Permission denied"));
            })
        } else if(window.ondevicemotion !== undefined) {
            assert();
        } else {
            reject(new Error("DeviceMotion is not supported."));
        }
    })
}

async function requestSensor(constraint=undefined) {
    return new Promise((resolve, reject) => {
        if(__SENSOR__._requested){
            resolve(__SENSOR__);
            return __SENSOR__;
        }
        if(constraint == undefined) constraint = DEFAULT_CONSTRAINT;

        const media_promise = new Promise((resolve, reject) => {
            navigator.mediaDevices.getUserMedia(constraint)
            .then(stream => { 
                __SENSOR__.stream = stream; 
                __SENSOR__.track = stream.getTracks()[0];
                if(typeof ImageCapture === 'undefined') {
                    __SENSOR__.video_el = document.createElement('__video_element_for_imacap__');
                    __SENSOR__.video_el.srcObject = stream;
                } else {
                    __SENSOR__.imageCapture = new ImageCapture(__SENSOR__.track);
                }
                resolve();
            })
            .catch(err => {
                console.error(err);
                alert(err);
                reject(new Error("Failed to get media stream"));
            });   
        });
        
        const motion_promise = new Promise((resolve, reject) => {
            requestMotion().then(() => {
                window.addEventListener('devicemotion', (event) => {
                    __SENSOR__.accelerate = event.acceleration;
                });
                window.addEventListener('deviceorientation', (event) => {
                    __SENSOR__.gyroscope = {
                        alpha: event.alpha,
                        beta: event.beta,
                        gamma: event.gamma
                    }
                });
            })
            .catch(err => {
                console.error(err);
                alert(err);
            })
            .finally(resolve());
        });

        Promise.all([media_promise, motion_promise])
        .then(() => {
            __SENSOR__._requested = true;
            resolve(__SENSOR__);
        })
    })
}

export default requestSensor;