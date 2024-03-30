import requestSensor from "./js/sensor.js"
const worker = new Worker("./js/svo_pro.js", { type: "module" });
var sensor = null;

const Session = {
    inited_: false,
    started_: false,
    use_imu_: false,
    viewPose_: [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ],
    imu_loop_id_: null,
    frame_loop_id_: null,
    start: function(fps=30, imuHz=200) {
        if(!this.inited_) return;
        if(this.started_) return;

        if(this.use_imu_) {
            this.imu_loop_id_ = setInterval(() => {
                var imudata = sensor.getMotion(true);
                imudata.name = "addMotion";
                worker.postMessage(imudata);
            }, 1000 / imuHz);
        }
        
        this.frame_loop_id = setInterval(async () => {
            // worker.postMessage({name:'getViewPose'})
            try{
                let res = await sensor.getFrameBitmap();
                res.name = 'addFrameBitmap'
                worker.postMessage(res, [res.data])
            }catch(err) { 
                // ignore 
            }
        }, 1000 / fps);
        
        this.started_ = true;
    },
    end: function() {
        if(!this.started_) return;
        worker.postMessage({name: "reset"});
        clearInterval(this.imu_loop_id_);
        clearInterval(this.frame_loop_id_);
    },
    getViewPose: function() {
        return this.viewPose_;
    },
    getStream: function() {
        return sensor.getStream();
    }
}

worker.onmessage = e => {
    switch (e.data.name) {
        case "setViewPose": {
            Session.viewPose_ = e.data.data; 
            break;
        }
        case "init": {
            Session.inited_ = e.data.data;
            if(!Session.inited_) console.error(e.data.msg);
            break;
        }    
     }
}

worker.onerror = e => console.error(e);

function terminate() {
    worker.terminate();
}


async function requestSession(settings={}) {
    return new Promise(async (resolve, reject) => {
        // check inited
        if(Session.inited_) {
            resolve(Session);
            return;
        }
        sensor = await requestSensor();
        // merge settings
        let default_settings = {
            use_imu: false,
            width: 640,
            height: 480    
        }
        for(const key in settings) {
            default_settings[key] = settings[key];
        }
        // initialize
        Session.use_imu_ = default_settings.use_imu;
        worker.postMessage({
            name: "init",
            data: default_settings
        });
        // checking init state
        var looptime = 0;
        var loopid = setInterval(() => {
            looptime += 1000;
            if(Session.inited_) {
                clearInterval(loopid);
                resolve(Session);
            } else if(looptime >= 6000) {
                clearInterval(loopid);
                reject(new Error("Initialization failed: timeout."));
            }
        }, 1000);
    })
}

export default { requestSession, terminate }