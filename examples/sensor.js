class Sensor{
    _requested = false;
    // motion
    accelerate = null;
    gyroscope = null;
    // media
    stream = null;
    track = null;
    imageCapture = null;

    getMotion(arrayLike=false) {
        return {
            accelerate: arrayLike ? [this.accelerate.x, this.accelerate.y, this.accelerate.z] : this.accelerate,
            gyroscope: arrayLike ? [this.gyroscope.alpha, this.gyroscope.beta, this.gyroscope.gamma] : this.gyroscope,
            timestamp: Date.now()
        }
    }

    async getFrame() {
        const bitmap = await this.imageCapture.grabFrame();
        const canvas = new OffscreenCanvas(bitmap.width, bitmap.height);
        const ctx = canvas.getContext('2d');
        ctx.drawImage(bitmap, 0, 0);
        return {
            data: ctx.getImageData(0, 0, bitmap.width, bitmap.height).data,
            timestamp: Date.now()
        }
    }

    getTrack() {
        return this.track;
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
            if(window.DeviceMotionEvent === undefined) {
                reject(new Error("DeviceMotion is not supported."));
                return;
            }
            if(window.DeviceOrientationEvent === undefined) {
                reject(new Error("DeviceOrientation is not supported."));
                return;
            }
            resolve();
        }

        if(typeof window.DeviceMotionEvent.requestPermission === 'function') {
            window.DeviceMotionEvent.requestPermission()
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

async function requestSensor(constraint) {
    return new Promise((resolve, reject) => {
        if(__SENSOR__._requested){
            resolve(__SENSOR__);
            return __SENSOR__;
        }
        if(constraint === undefined) constraint = DEFAULT_CONSTRAINT;

        const media_promise = new Promise((resolve, reject) => {
            navigator.mediaDevices.getUserMedia(constraint)
            .then(stream => { 
                __SENSOR__.stream = stream; 
                __SENSOR__.track = stream.getTracks()[0];
                __SENSOR__.imageCapture = new ImageCapture(__SENSOR__.track);
            })
            .catch(err => console.error(err));
            resolve();
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
            .catch(err => console.error(err));
            resolve();
        });

        Promise.all([media_promise, motion_promise])
        .then(() => {
            __SENSOR__._requested = true;
            resolve(__SENSOR__);
        })
    })
}

export { requestSensor };