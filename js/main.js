/*
    Nikolay Belov
    -------------

    Remote control for Syma S107/S111 and S026 IR Helicopters.
    Works with Arduino based IR transmitter(with HM-10 module) via BLE.

    Inspired me: https://habr.com/ru/post/339146/
     =======================================================================================

    Packet to Arduino for S107G/S111G:
    ----------------------------------
    HHH TTT YYY PPP AAA SSSS\n - 20 байт
    HHH = 107
    TTT - Throttle          0..127
    YYY - Yaw               0..127
    PPP - Pitch             0..127
    AAA - Trim              0..127
    SSSS - control sum (HHH + TTT + YYY + PPP + AAA)
    
    Packet to Arduino for S026:
    ---------------------------
    HHH TTT YYY PP B AAA SSSS\n - 20 байт
    HHH = 026
    TTT - Throttle          0..127
    YYY - Yaw               0..63
    PP  - Pitch             0..15
    B   - Buttons           0..3
    AAA - Trim              0..63   // 0..31
    SSSS - control sum (HHH + TTT + YYY + PP + B + AAA)

    Packet from Arduino:
    --------------------
    current parameters:
    107 TTT YYY PPP VVV SSSS\n - 20 байт
    or
    026 TTT YYY PP B VVV SSSS\n

    VVV - battery level (512 -> 5V)

    If connection to Arduino via BLE is established this application receiving 
    status packets every 500 mS.
    Application must response to every packet with own packed with actual parameters.

    If user changes control parameters, control packet will be sent immediately.
*/

// Joystick parameters
const joystickSize = 160;
const deadZone = 16;

// Helicopter parameters
const copter107 = '107';
const copter111 = '111';
const copter026 = '026';

const maxThrottle = 127;

const maxYaw107 = 127;
const maxPitch107 = 127;
const maxTrim107 = 127;
const stepYaw107 = 1;
const stepPitch107 = 1;
const stepTrim107 = 1;

const maxYaw026 = 63;
const maxPitch026 = 15;
const maxTrim026 = 63;
const stepYaw026 = 0.5;
const stepPitch026 = 0.125;
const stepTrim026 = 0.5;

// Init parameters
var copterType = copter107;

var maxYaw = maxYaw107;
var maxPitch = maxPitch107;
var maxTrim = maxTrim107;

var halfYaw = (maxYaw - 1) / 2;
var halfPitch = (maxPitch - 1) / 2;
var halfTrim = (maxTrim - 1) / 2;

var stepYaw = stepYaw107;      
var stepPitch = stepPitch107;
var stepTrim = stepTrim107;

const kBattery = 5.0 / 512;
var statBattery = 9.0;

var setThrottle = 0;
var setYaw = halfYaw;
var setPitch = halfPitch;
var setTrim = halfTrim;
var setButt = 0;

var statThrottle = 0;
var statYaw = 0;
var statPitch = 0;
var statTrim = 0;
var statButt = 0;

// Object cache of the selected device
let deviceCache = null;
// Feature Object Cache
let characteristicCache = null;

/* ============================== BLE ============================= */

function connect() {
    var ret = (deviceCache ? Promise.resolve(deviceCache) :
        requestBluetoothDevice()).
        then(device => connectDeviceAndCacheCharacteristic(device)).
        then(characteristic => startNotifications(characteristic)).
        catch(error => document.getElementById("togglesw").checked = false);
        //catch(error => resetParameters());

    if (ret) {
        document.getElementById('rmark_motl').style.visibility = 'visible';
        document.getElementById('rmark_motr').style.visibility = 'visible';
        document.getElementById('rmark_ser').style.visibility = 'visible';

        //cleanScreen();
        //writeToScreen('== BLE connected ==');
    }
    return ret;
}

// Device request
function requestBluetoothDevice() {
    //
    //writeToScreen('Requesting bluetooth device...');

    return navigator.bluetooth.requestDevice({
        filters: [{services: [0xFFE0]}],
    }).
        then(device => {
            //writeToScreen('"' + device.name + '" bluetooth device selected');
            deviceCache = device;

            // Добавленная строка
            deviceCache.addEventListener('gattserverdisconnected',
                handleDisconnection);

            return deviceCache;
        });
}

function handleDisconnection(event) {
    let device = event.target;

    //writeToScreen('"' + device.name +
    //    '" bluetooth device disconnected, trying to reconnect...');

    connectDeviceAndCacheCharacteristic(device).
        then(characteristic => startNotifications(characteristic)).
        catch(error => resetParameters());
        //catch(error => log(error));
    // сбрасываем togglesw
    //document.getElementById("togglesw").checked = false;
}

function disconnect() {
    if (deviceCache) {
        //writeToScreen('Disconnecting from "' + deviceCache.name + '" bluetooth device...');
        deviceCache.removeEventListener('gattserverdisconnected',
            handleDisconnection);

        if (deviceCache.gatt.connected) {
            deviceCache.gatt.disconnect();
            //writeToScreen('"' + deviceCache.name + '" bluetooth device disconnected');
        }
        else {
            //writeToScreen('"' + deviceCache.name +
            //    '" bluetooth device is already disconnected');
        }
    }

     // added condition
    if (characteristicCache) {
        characteristicCache.removeEventListener('characteristicvaluechanged',
            handleCharacteristicValueChanged);
        characteristicCache = null;
    }

    deviceCache = null;
}

// Connect to device and get the characteristic
function connectDeviceAndCacheCharacteristic(device) {
    //
    if (device.gatt.connected && characteristicCache) {
        return Promise.resolve(characteristicCache);
    }

    //writeToScreen('Connecting to GATT server...');

    return device.gatt.connect().
        then(server => {
            //writeToScreen('GATT server connected, getting service...');

            return server.getPrimaryService(0xFFE0);
        }).
        then(service => {
            //writeToScreen('Service found, getting characteristic...');

            return service.getCharacteristic(0xFFE1);
        }).
        then(characteristic => {
            //writeToScreen('Characteristic found');
            characteristicCache = characteristic;

            return characteristicCache;
        });
}

// Enable getting characteristic changes notification
function startNotifications(characteristic) {
    //
    //writeToScreen('Starting notifications...');

    return characteristic.startNotifications().
        then(() => {
            //writeToScreen('Notifications started');
            // added string
            characteristic.addEventListener('characteristicvaluechanged',
                handleCharacteristicValueChanged);
        });
}


/* ================ Throttle, Pitch and Yaw Indicators ================ */
function Gauge(el) {

        // ##### Private Properties and Attributes

        var element,            // Containing element for the info component
                data,           // `.gauge--data` element
                needle,         // `.gauge--needle` element
                value = 0.0,    // Current gauge value from 0 to 1
                prop;           // Style for transform

        // ##### Private Methods and Functions

        var setElement = function(el) {
                // Keep a reference to the various elements and sub-elements
                element = el;
                data = element.querySelector(".gauge--data");
                needle = element.querySelector(".gauge--needle");
        };
        var setValue = function(x) {
                value = x;
                var turns = -0.5 + (x * 0.5);
                data.style[prop] = "rotate(" + turns + "turn)";
                needle.style[prop] = "rotate(" + turns + "turn)";
        };

        // ##### Object to be Returned

        function exports() { };

        // ##### Public API Methods

        exports.element = function(el) {
                if (!arguments.length) { return element; }
                setElement(el);
                return this;
        };

        exports.value = function(x) {
                if (!arguments.length) { return value; }
                setValue(x);
                return this;
        };
                
        // ##### Initialization
                
        var body = document.getElementsByTagName("body")[0];
        ["webkitTransform", "mozTransform", "msTransform", "oTransform", "transform"].
                forEach(function(p) {
                        if (typeof body.style[p] !== "undefined") { prop = p; }
                });
      
        if (arguments.length) {
                setElement(el);
        }
      
        return exports;
};

// Create indicators
var gaugepi = new Gauge(document.getElementById("gaugepi"));
var gaugeya = new Gauge(document.getElementById("gaugeya"));
var gaugeth = new Gauge(document.getElementById("gaugeth"));

function gaugeUpdate() {

    var tt = setPitch / (maxPitch - 1);
    if (tt > 1) {
        tt = 1;
    }
    gaugepi.value(tt);
    tt = setYaw / (maxYaw - 1);
    if (tt > 1) {
        tt = 1;
    }
    gaugeya.value(tt);
    gaugeth.value(setThrottle / maxThrottle);

    document.getElementById('lthrottle').innerHTML = 
            'Throttle:<br>' + setThrottle + '<br>(' + statThrottle + ')';
    document.getElementById('lpitch').innerHTML = 
            'Pitch:<br>' + setPitch + '<br>(' + statPitch + ')';
    document.getElementById('lyaw').innerHTML = 
            'Yaw:<br>' + setYaw + '<br>(' + statYaw + ')';
};

gaugeUpdate();


var win_width = window.screen.availWidth;
document.body.style.width = win_width;
var win_height = window.screen.availHeight;

/* =================== Change helicopter type =================== */
function copterChange(value) {
    if ((value === 'Syma S111G') || (value === 'Syma S107G')) {
        if (value === 'Syma S107G') {
            document.body.style.backgroundImage = "url('img/all_037a_015.jpg')";
        } else {
            document.body.style.backgroundImage = "url('img/cockpit1024.jpg')";
        }
        copterType = copter107;
        maxYaw = maxYaw107;
        maxPitch = maxPitch107;
        maxTrim = maxTrim107;
        stepYaw = stepYaw107;      
        stepPitch = stepPitch107;
        stepTrim = stepTrim107;
    } else if (value === 'Syma S026G') {
        document.body.style.backgroundImage = "url('img/ch47f.png')";
        copterType = copter026;
        maxYaw = maxYaw026;
        maxPitch = maxPitch026;
        maxTrim = maxTrim026;
        stepYaw = stepYaw026;      
        stepPitch = stepPitch026;
        stepTrim = stepTrim026;        
    }

    halfYaw = (maxYaw - 1) / 2;
    halfPitch = (maxPitch - 1) / 2;
    halfTrim = (maxTrim - 1) / 2;
    
    setThrottle = 0;
    setYaw = halfYaw;
    setPitch = halfPitch;
    setTrim = halfTrim;
    setButt = 0;

    document.getElementById('maxpitch').innerHTML = maxPitch;
    document.getElementById('maxyaw').innerHTML = maxYaw;

    document.getElementById('throttle').value = setThrottle;
    gaugeUpdate();

    sendToBLE();
}

function resetParameters() {
    document.getElementById("togglesw").checked = false;
}

// Receiving packet from Arduino
function handleCharacteristicValueChanged(event) {
    let value = new TextDecoder().decode(event.target.value);
    //writeToScreen('rec: ' + value);

    var rType = parseInt(value.substring(0, 3));
    statThrottle = parseInt(value.substring(3, 6));
    statYaw = parseInt(value.substring(6, 9));
    if (rType === 107) {
        statPitch = parseInt(value.substring(9, 12));
        statButt = 0;
    }
    else if (rType === 26) {
        statPitch = parseInt(value.substring(9, 11));
        statButt = parseInt(value.substring(11, 12));
    }
    var rBattery = parseInt(value.substring(12, 15));

    var psum = value.substring(15, 19);
    var isum = rType + statThrottle + statYaw + statPitch + statButt + rBattery;
    if (isum == psum) {
        document.getElementById('battery').value = rBattery * kBattery;
        gaugeUpdate();
    }
    
    // response to received packet
    sendToBLE();
}

// Enable / Disable connection to IR transmitter (Arduino)
function crtrl_on(sw) {
    // сброс параметров движения
    copterChange(document.getElementById('soflow-color').value);
    if (sw.checked) {
        // connect to BLE
        connect();
        ctrlFlag = true;
    }
    else {
        sendToBLE();
        disconnect();
        ctrlFlag = false;
    }
}

// send message via BLE
function doSend(message) {
    message = String(message);
    if (!message || !characteristicCache) {
        return;
    }
    writeToCharacteristic(characteristicCache, message);
    //writeToScreen('send: ' + message);
}

// write value to characteristic
function writeToCharacteristic(characteristic, data) {
    characteristic.writeValue(new TextEncoder().encode(data));
}

// for debug
const scrLen = 10;
function writeToScreen(message) {
    //var outputEl = document.getElementById('diagmsg');
    /*
    if (outputEl.children.length == scrLen) {
        outputEl.removeChild(outputEl.children[0]);
    }
    
    outputEl.insertAdjacentHTML('beforeend',
      '<div class="myterm">' + message + '</div>');
      */
    //outputEl.innerHTML = message;
    document.getElementById("diagmsg").innerHTML = message;
}

function cleanScreen() {
    var outputEl = document.getElementById('diagmsg');
    var l = outputEl.children.length;
    for (var i = l - 1; i >= 0; i--) {
        outputEl.removeChild(outputEl.children[i]);
    }    
}

// Make package for IR transmitter and send via BLE
function sendToBLE() {

    var st = '';
    var is = 0;
    var ss = '';

    var ist = parseInt(setThrottle) + parseInt(setYaw) + parseInt(setPitch) + parseInt(setTrim);

    if ((copterType === copter107) || (copterType === copter111)) {
        is = 107 + ist;
        st = '107' + ('000' + setThrottle).slice(-3) +
            ('000' + setYaw).slice(-3) +
            ('000' + setPitch).slice(-3) +
            ('000' + setTrim).slice(-3);
    } else {
        is = 26 + ist + parseInt(setButt);
        st = '026' + ('000' + setThrottle).slice(-3) +
            ('000' + setYaw).slice(-3) +
            ('00' + setPitch).slice(-2) +
            ('0' + setButt).slice(-1) +
            ('000' + setTrim).slice(-3);
    }
    ss = ('000' + is).slice(-4);        
    st = st + ss;

    doSend(st + '\n');
    console.log('Send: ' + st);
};


/* ==================== Yaw & Pitch JoyStick ==================== */
// previous touch - distance from joystick center
var joyX = 0;
var joyY = 0;

// Create joystick
var joystickYawPitch = nipplejs.create({
    zone: document.getElementById('pitchyaw'),
    multitouch: false,
    maxNumberOfNipples: 1,
    mode: "dynamic",
    color: 'blue',
    size: joystickSize
});

joystickYawPitch.on('move', function (evt, nipple) {
    // x, y - px, integer 
    var x = nipple.position.x - nipple.instance.position.x; // > 0 - yaw right
    var y = nipple.position.y - nipple.instance.position.y; // > 0 = pitch backward

    var flMove = false;
    if (Math.abs(x) <= deadZone) {
        x = 0;
    } else {
        if (x > 0) {
            x = x - deadZone;
        } else {
            x = x + deadZone;
        }
        flMove = true;
    }

    if (Math.abs(y) <= deadZone) {
        y = 0;
    } else {
        if (y > 0) {
            y = y - deadZone;
        } else {
            y = y + deadZone;
        }
        flMove = true;
    }

    if (flMove) {
        flMove = false;
        if ( (joyY !== y) || (joyX !== x) ) {   // moving
            flMove = true;

            setPitch = halfPitch - Math.round(y * stepPitch);  // > 0 - backward
            if (setPitch > maxPitch) {
                setPitch = maxPitch;
            } else if (setPitch < 0) {
                setPitch = 0;
            }
            joyY = y;

            setYaw = halfYaw + Math.round(x * stepYaw);     // > 0 - right
            if (setYaw > maxYaw) {
                setYaw = maxYaw;
            } else if (setYaw < 0) {
                setYaw = 0;
            }
            joyX = x;            
        }
        
    }

    if (flMove) {
        console.log('x=' + x + ' pitch=' + setPitch + ' y=' + y + ' yaw=' + setYaw);
        gaugeUpdate();
        sendToBLE();
    }
});

joystickYawPitch.on('end', function () {
    setPitch = halfPitch;
    setYaw = halfYaw;
    gaugeUpdate();      // indicators update
    sendToBLE();        // send package to IR transmitter
});

// Calculate current joystick center coordinates
function calcContCenter(cont) {
    var bodyRect = document.body.getBoundingClientRect();
    var contRect = document.getElementById(cont).getBoundingClientRect();
    var offsetTop   = contRect.top - bodyRect.top;
    var offsetLeft   = contRect.left - bodyRect.left;
    var centerXY = new Object();
    centerXY['x'] = Math.round(offsetLeft + contRect.width / 2);
    centerXY['y'] = Math.round(offsetTop + contRect.height / 2);
    return centerXY;
}

// Throttle slider value were changed
function throttleInput(value) {
    document.getElementById('lthrottle').innerHTML = 'Throttle: ' + value;
    setThrottle = value;
    gaugeUpdate();      // indicators update
    sendToBLE();        // send package to IR transmitter
}

function resize_on() {

}
