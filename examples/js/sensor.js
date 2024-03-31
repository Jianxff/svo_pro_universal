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
            var source;
            if(this.imageCapture) {
                source = await this.imageCapture.grabFrame();
            } else {
                source = this.video_el;
            }
            if(!this.context) {
                this.context = new OffscreenCanvas(source.width, source.height).getContext('2d', { willReadFrequently: true });
            }
            this.context.drawImage(source, 0, 0);
            return {
                data: this.context.getImageData(0, 0, source.width, source.height), // ImageData
                timestamp: Date.now()
            }
        }catch(err){
            // ignore
        }
        return null;
    }

    async getFrameBitmap() {
        var bitmap;
        if(this.imageCapture) {
            bitmap = await this.imageCapture.grabFrame();
        } else {
            console.error('Transfer of bitmap is not supported.')
            alert('Transfer of bitmap is not supported.')
            return undefined
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

        if(window.DeviceMotionEvent.requestPermission !== undefined) {
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
                if(typeof ImageCapture !== 'function') {
                    console.warn('ImageCapture is not supported.')
                    const video = document.createElement('video');
                    video.srcObject = stream;
                    video.setAttribute( 'autoplay', 'autoplay' );
                    video.setAttribute( 'playsinline', 'playsinline' );
                    video.setAttribute( 'webkit-playsinline', 'webkit-playsinline' );
                    video.width = constraint.video.width;
                    video.height = constraint.video.height;
                    __SENSOR__.video_el = video;
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