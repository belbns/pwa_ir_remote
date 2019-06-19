/*

    Формат пакета к Ардуино для S107G/S111G:
    HHHTTTYYYPPPMMMSSSS\n
    HHH = 107
    TTT - Throttle          0..127
    YYY - Yaw               0..127
    PPP - Pitch             0..127
    MMM - Trim              0..127
    SSS - контрольная сумма HHH + TTT + YYY + PPP + MMM (последние 4 знака)
    
    для S026:
    HHHTTTYYYPPBMMMSSSS\n
    HHH = 026
    TTT - Throttle          0..127
    YYY - Yaw               0..63
    PP  - Pitch             0..16
    B   - Buttons           0..2
    MMM - Trim              0..31
    SSS - контрольная сумма HHH + TTT + YYY + PP + B + MMM (последние 4 знака)

    запрос состояния:
    9990000000000000999\n

    Пакет от Ардуино:

    состояние батареи
    999BBBB00000000BBBBB\n

    текущие параметры:
    107TTTYYYPPPMMMSSSS\n
    или
    026TTTYYYPPBMMMSSSS\n

*/

// переменные управления мобильной платформой
var ctrlLeds = [0, 0, 0, 0];
var LedsQueue = 0;
var Dist = 2048;
var DistQueue = 0;
var CmdQueue = 0;
var adc_val = [0, 0, 0, 0];
var LowBatt = false;
var HighLoad = false;

const adcBattMin = 1580;
const battMeterMax = 120;



/*
var storage = window.localStorage;

var remote_ip = storage.getItem("remote_ip");
if (!remote_ip) {
    //remote_ip = "192.168.4.22";
    remote_ip = "127.0.0.1";
    storage.setItem("remote_ip", remote_ip);
}

var remote_port = storage.getItem("remote_port");
if (!remote_port) {
    remote_port = 2012;
    storage.setItem("remote_port", remote_port);
}
*/

// Кэш объекта выбранного устройства
let deviceCache = null;
// Кэш объекта характеристики
let characteristicCache = null;

// Запустить выбор Bluetooth устройства и подключиться к выбранному
function connect() {
    var ret = (deviceCache ? Promise.resolve(deviceCache) :
        requestBluetoothDevice()).
        then(device => connectDeviceAndCacheCharacteristic(device)).
        then(characteristic => startNotifications(characteristic)).
        catch(error => writeToScreen(error));

    if (ret) {
        document.getElementById('rmark_motl').style.visibility = 'visible';
        document.getElementById('rmark_motr').style.visibility = 'visible';
        document.getElementById('rmark_ser').style.visibility = 'visible';

        cleanScreen();
        writeToScreen('== BLE connected ==');
    }
    return ret;
}

// Запрос выбора Bluetooth устройства
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

// Обработчик разъединения
function handleDisconnection(event) {
    let device = event.target;

    //writeToScreen('"' + device.name +
    //    '" bluetooth device disconnected, trying to reconnect...');

    connectDeviceAndCacheCharacteristic(device).
        then(characteristic => startNotifications(characteristic)).
        catch(error => log(error));
}

// Отключиться от подключенного устройства
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

     // Добавленное условие
    if (characteristicCache) {
        characteristicCache.removeEventListener('characteristicvaluechanged',
            handleCharacteristicValueChanged);
        characteristicCache = null;
    }

    deviceCache = null;

    document.getElementById('rmark_motl').style.visibility = 'hidden';
    document.getElementById('rmark_motr').style.visibility = 'hidden';
    document.getElementById('rmark_ser').style.visibility = 'hidden';

}

// Подключение к определенному устройству, получение сервиса и характеристики
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

// Включение получения уведомлений об изменении характеристики
function startNotifications(characteristic) {
    //
    //writeToScreen('Starting notifications...');

    return characteristic.startNotifications().
        then(() => {
            //writeToScreen('Notifications started');
            // Добавленная строка
            characteristic.addEventListener('characteristicvaluechanged',
                handleCharacteristicValueChanged);
        });
}


function Gauge(el) {

        // ##### Private Properties and Attributes

        var element,      // Containing element for the info component
                data,         // `.gauge--data` element
                needle,       // `.gauge--needle` element
                value = 0.0,  // Current gauge value from 0 to 1
                prop;         // Style for transform

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


/* Получение данных
    Пакет от Ардуино:
    состояние батареи
    999BBBB00000000BBBBB\n
    текущие параметры:
    107TTTYYYPPPMMMSSSS\n
    или
    026TTTYYYPPBMMMSSSS\n
*/
const joystickSize = 192;
const deadZone = 32;

//
const maxThrottle = 127;

const maxYaw107 = 127;
const maxPitch107 = 127;
const maxTrim107 = 127;
const stepYaw107 = 1;
const stepPitch107 = 1;
const stepTrim107 = 1;

const maxYaw026 = 63;
const maxPitch026 = 15;
const maxTrim026 = 31;
const stepYaw026 = 2;
const stepPitch026 = 8;
const stepTrim026 = 4;

const copter107 = '107';
const copter026 = '026';

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

//var distPitchYaw = 0;
//var dirPitchYaw = 0;
var joyX = 0;
var joyY = 0;

var statBattery = 9.0;

var setThrottle = 0;
var setYaw = halfYaw;
var setPitch = halfPitch;
var setTrim = halfTrim;
var setButt = 0;

var gaugepi = new Gauge(document.getElementById("gaugepi"));
var gaugeya = new Gauge(document.getElementById("gaugeya"));
var gaugeth = new Gauge(document.getElementById("gaugeth"));

var win_height = window.screen.availHeight;
var win_width = window.screen.availWidth;

//document.body.style.width = win_width;
//document.body.style.height = 600 + 'px';
//document.html.style.height = 600 + 'px';

writeToScreen('height: ' + win_height + '   width: ' + win_width);


function gaugeUpdate() {
    gaugepi.value(setPitch / maxPitch);
    gaugeya.value(setYaw / maxYaw);
    gaugeth.value(setThrottle / maxThrottle);

    document.getElementById('lthrottle').innerHTML = 'Throttle: ' + setThrottle;
    document.getElementById('lpitch').innerHTML = 'Pitch: ' + (setPitch - halfPitch);
    document.getElementById('lyaw').innerHTML = 'Yaw: ' + (setYaw - halfYaw);
};

function copterChange(value) {
    if (value === 'Syma S107G') {
        if (copterType != copter107) {
            copterType = copter107;
            maxYaw = maxYaw107;
            maxPitch = maxPitch107;
            maxTrim = maxTrim107;
            stepYaw = stepYaw107;      
            stepPitch = stepPitch107;
            stepTrim = stepTrim107;
        }
    } else if (value === 'Syma S026G') {
        if (copterType != copter026) {
            copterType = copter026;
            maxYaw = maxYaw026;
            maxPitch = maxPitch026;
            maxTrim = maxTrim026;
            stepYaw = stepYaw026;      
            stepPitch = stepPitch026;
            stepTrim = stepTrim026;
        }
    }

    halfYaw = (maxYaw - 1) / 2;
    halfPitch = (maxPitch - 1) / 2;
    halfTrim = (maxTrim - 1) / 2;
    
    setThrottle = 0;
    setYaw = halfYaw;
    setPitch = halfPitch;
    setTrim = halfTrim;
    setButt = 0;
    document.getElementById('throttle').value = setThrottle;
    gaugeUpdate();
    sendToBLE(copterType);

/*
    getComputedStyle(document.documentElement).getPropertyValue('--my-variable-name');
    document.documentElement.style.setProperty('--my-variable-name', 'pink');
*/
}


function handleCharacteristicValueChanged(event) {
    let value = new TextDecoder().decode(event.target.value);
    writeToScreen('rec: ' + value);

    var rThrottle = 0;
    var rYaw = 0;
    var rPitch = 0;
    var rTrim = 0;
    var rButt = 0;
    var rBattery

    if (value.length >= 19) {
        var psum = value.substr(15, 19);
        var isum = 0;
        
        var ptype = parseInt(value.substr(0, 3));
        if (ptype === 107) {
            rThrottle = parseInt(value.substr(3, 6));
            rYaw = parseInt(value.substr(6, 9));
            rPitch = parseInt(value.substr(9, 12));
            rTrim = parseInt(value.substr(12, 15));
        }
        else if (ptype === 026) {
            rThrottle = parseInt(value.substr(3, 6));
            rYaw = parseInt(value.substr(6, 9));
            rPitch = parseInt(value.substr(9, 11));
            rButt = parseInt(value.substr(11, 12));
            rTrim = parseInt(value.substr(12, 15));
        }
        else if (ptype === 999) {
            rBattery = parseInt(value.substr(3, 7));
            rThrottle = 0;
            rYaw = 0;
            rPitch = 0;
            rTrim = 0;
        }
        else {
            isum = -1;
        }

        if (isum === 0) {
            var isum1 = ptype +  rBattery + rThrottle + rYaw + rPitch + rTrim + rButt;
            var sSum = isum1.toString;
            if (psum === sSum.substr(sSum.length -4, sSum.length)) {
                if (ptype === '999') {
                    statBattery = rBattery;
                    meterb.setAttribute('value', (statBattery / 102.4).toString);
                }
                else {
                    statThrottle = rThrottle;
                    statYaw = rYaw;
                    statPitch = rPitch;
                    statTrim = rTrim;
                    statButt = rButt;
                }

            }
        }
    }
}

function crtrl_on(sw) {
    if (sw.checked) {
        // connect to BLE
        //connect();
        ctrlFlag = true;
        writeToScreen("switch on");
    }
    else {
        //disconnect();
        ctrlFlag = false;
        writeToScreen("switch off");
    }
}


function doSend(message) {
    message = String(message);
    if (!message || !characteristicCache) {
        return;
    }
    writeToCharacteristic(characteristicCache, message);
    writeToScreen('send: ' + message);
}

// Записать значение в характеристику
function writeToCharacteristic(characteristic, data) {
    characteristic.writeValue(new TextEncoder().encode(data));
}

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

//function sendToBLE(token, newcmd, par1, devnum) {
function sendToBLE(ctype) {

    var st = '---';
    if (ctype === '107') {
        var is = parseInt(ctype) + setThrottle + setYaw + setPitch + setTrim;
        var ss = ('000' + is).slice(-4);
        //var s4 = ss.slice(-4);
        st = ctype + ('000' + setThrottle).slice(-3) +
            ('000' + setYaw).slice(-3) +
            ('000' + setPitch).slice(-3) +
            ('000' + setTrim).slice(-3) + ss + '\n';
    }

    if (st !== '---') {
        doSend(st + '\n');
        console.log('Send: ' + st);
    }    

    return true;
};

gaugeUpdate();

var joystickYawPitch = nipplejs.create({
    zone: document.getElementById('pitchyaw'),
    multitouch: false,
    maxNumberOfNipples: 1,
    mode: "dynamic",
    color: 'blue',
    size: joystickSize
});

joystickYawPitch.on('move', function (evt, nipple) {

    var ndir = nipple.angle.radian;
    var ndist= nipple.distance;
    //console.log('ndist=' + ndist + '  ndir=' + ndir);
    var flMove = false;

    var x = ndist * Math.cos(ndir);
    if (Math.abs(x) < deadZone) {
        x = 0;
    } else {
        if (x > 0) {
            x = x - deadZone;
        } else {
            x = x + deadZone;
        }
        flMove = true;
    }
    var y = ndist * Math.sin(ndir);
    if (Math.abs(y) < deadZone) {
        y = 0;
    } else {
        if (y > 0) {
            y = y - deadZone;
        } else {
            y = y + deadZone;
        }
        flMove = true;
    }
    //console.log('x=' + x + '  y=' + y);

    if (flMove) {
        flMove = false;
        if ( (Math.abs(joyY - y) >= 1) || (Math.abs(joyX - x) >= 1) ) {
            pitch = halfPitch + Math.round(y) * stepPitch;
            if (pitch > maxPitch) {
                pitch = maxPitch;
            } else if (pitch < 0) {
                pitch = 0;
            }
            joyY = y;
            setPitch = pitch;

            yaw = halfYaw + Math.round(x) * stepYaw;
            if (yaw > maxYaw) {
                yaw = maxYaw;
            } else if (yaw < 0) {
                yaw = 0;
            }
            joyX = x;
            setYaw = yaw;
            flMove = true;
        }
    }

    if (flMove) {
        gaugeUpdate();
        sendToBLE(copterType);
    }
});

joystickYawPitch.on('end', function () {
    setPitch = halfPitch;
    setYaw = halfYaw;
    gaugeUpdate();
    sendToBLE(copterType);
});

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

function throttleInput(value) {
    document.getElementById('lthrottle').innerHTML = 'Throttle: ' + value;
    setThrottle = value;
    gaugeUpdate();
    sendToBLE(copterType);
}

function resize_on() {

}

