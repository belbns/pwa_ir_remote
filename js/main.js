/*
    Пульт управления вертолетами с IR излучателем на базе Ардуино c модулем HM-10 через BLE
    =======================================================================================

    Формат пакета к Ардуино для S107G/S111G:
                            ---------------
    HHH TTT YYY PPP MMM SSSS\n - 20 байт
    HHH = 107
    TTT - Throttle          0..127
    YYY - Yaw               0..127
    PPP - Pitch             0..127
    MMM - Trim              0..127
    SSSS - контрольная сумма HHH + TTT + YYY + PPP + MMM
    
    для S026:
    --------
    HHH TTT YYY PP B MMM SSSS\n - 20 байт
    HHH = 026
    TTT - Throttle          0..127
    YYY - Yaw               0..63
    PP  - Pitch             0..16
    B   - Buttons           0..2
    MMM - Trim              0..31
    SSSS - контрольная сумма HHH + TTT + YYY + PP + B + MMM

    Пакет от Ардуино:
          ----------
    текущие параметры:
    107 TTT YYY PPP VVV SSSS\n - 20 байт
    или
    026 TTT YYY PP B VVV SSSS\n

    VVV - напряжение батареи 512 -> 5 вольт

    После установления соединения c Ардуино через BLE
    с интервалом в 500мС приходят пакеты с состоянием.
    Данный пульт должен в ответ посылать пакет с подтверждением текущих параметров,
    отсутствие 4-х пакетов подряд воспринимается как потеря управления и 
    приводит к сбросу параметров движения, передаваемых вертолету.
    
    Изменение параметров через органы управления вызывают немедленную отправку пакета.
*/


const joystickSize = 160;
const deadZone = 16;


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
const stepYaw026 = 0.5;
const stepPitch026 = 0.125;
const stepTrim026 = 0.25;

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

const kBattery = 5.0 / 512;
var statBattery = 9.0;

var setThrottle = 0;
var setYaw = halfYaw;
var setPitch = halfPitch;
var setTrim = halfTrim;
var setButt = 0;


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

/* ============================== BLE =============================

* Запустить выбор Bluetooth устройства и подключиться к выбранному
*/
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
        catch(error => resetParameters());
        //catch(error => log(error));
    // сбрасываем togglesw
    //document.getElementById("togglesw").checked = false;
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

    // copterChange(document.getElementById('soflow-color').value);

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


/* ================ индикаторы Throttle, Pitch, Yaw ================ */
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

var gaugepi = new Gauge(document.getElementById("gaugepi"));
var gaugeya = new Gauge(document.getElementById("gaugeya"));
var gaugeth = new Gauge(document.getElementById("gaugeth"));

function gaugeUpdate() {
/*
    gaugepi.value(setPitch / maxPitch);
    gaugeya.value(setYaw / maxYaw);
    gaugeth.value(setThrottle / maxThrottle);
*/
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

    document.getElementById('lthrottle').innerHTML = 'Throttle: ' + setThrottle;
    document.getElementById('lpitch').innerHTML = 'Pitch: ' + (setPitch);
    document.getElementById('lyaw').innerHTML = 'Yaw: ' + (setYaw);
};

gaugeUpdate();


var win_width = window.screen.availWidth;
document.body.style.width = win_width;
var win_height = window.screen.availHeight;
/*
document.body.style.height = 600 + 'px';
document.html.style.height = 600 + 'px';

var ctrlCont = document.getElementById('control-container');
var wh = document.documentElement.getBoundingClientRect().height; //ctrlCont.style.height;
var ww = document.documentElement.getBoundingClientRect().width;
var ch = ctrlCont.getBoundingClientRect().height;
var dpi = window.devicePixelRatio;
ctrlCont.style.top = (wh - ch) + 'px';  //91

writeToScreen('height: ' + wh + '   width: ' + ww + '  ch: ' + ch + '  dpi: ' + dpi);
*/

/* =================== обработка смены типа вертолета =================== */
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

function handleCharacteristicValueChanged(event) {
    let value = new TextDecoder().decode(event.target.value);
    //writeToScreen('rec: ' + value);

    var rThrottle = 0;
    var rYaw = 0;
    var rPitch = 0;
    var rButt = 0;
    var rBattery = 0;
    var rType = '';

    if (value.length >= 19) {
        rType = parseInt(value.substr(0, 3));
        rThrottle = parseInt(value.substr(3, 6));
        rYaw = parseInt(value.substr(6, 9));
        if (rType === 107) {
            rPitch = parseInt(value.substr(9, 12));
        }
        else if (rType === 26) {
            rPitch = parseInt(value.substr(9, 11));
            rButt = parseInt(value.substr(11, 12));
        }
        rBattery = parseInt(value.substr(12, 15));

        var psum = value.substr(15, 19);
        var isum = rType + rThrottle + rYaw + rPitch + rButt +  rBattery;
        if (isum == psum) {
            var batt_meter = document.getElementById('battery');
            batt_meter.value = rBattery * kBattery;
        }
        // на каждый принятый пакет отвечаем текущими параметрами
        sendToBLE();
    }
}

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


function doSend(message) {
    message = String(message);
    if (!message || !characteristicCache) {
        return;
    }
    writeToCharacteristic(characteristicCache, message);
    //writeToScreen('send: ' + message);
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
function sendToBLE() {

    var st = '-';
    var is = 0;
    var ss = '';

    if ((copterType === '107') || (copterType === '111')) {
        is = 107 + parseInt(setThrottle) + parseInt(setYaw) + parseInt(setPitch) + parseInt(setTrim);
        ss = ('000' + is).slice(-4);        
        st = copterType + ('000' + setThrottle).slice(-3) +
            ('000' + setYaw).slice(-3) +
            ('000' + setPitch).slice(-3) +
            ('000' + setTrim).slice(-3) + ss;
    } else if (copterType === '026') {
        is = 26 + setThrottle + setYaw + setPitch  + setButt + setTrim;
        ss = ('000' + is).slice(-4);        
        st = copterType + ('000' + setThrottle).slice(-3) +
            ('000' + setYaw).slice(-3) +
            ('00' + setPitch).slice(-2) +
            ('0' + setButt).slice(-1) +
            ('000' + setTrim).slice(-3) + ss;
    }

    if (st !== '-') {
        doSend(st + '\n');
        console.log('Send: ' + st);
    }    
};


/* ==================== JoyStick Yaw & Pitch ==================== */
// предыдущее касание - расстояния от центра
var joyX = 0;
var joyY = 0;

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
    var x = nipple.position.x - nipple.instance.position.x; // > 0 - yaw вправо
    var y = nipple.position.y - nipple.instance.position.y; // > 0 = pitch назад

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
        if ( (joyY !== y) || (joyX !== x) ) {   // было смещение
            flMove = true;
            setPitch = halfPitch - Math.round(y * stepPitch);  // > 0 - назад
            if (setPitch > maxPitch) {
                setPitch = maxPitch;
            } else if (setPitch < 0) {
                setPitch = 0;
            }
            joyY = y;

            setYaw = halfYaw + Math.round(x * stepYaw);     // > 0 - вправо
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
    gaugeUpdate();
    sendToBLE();
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
    sendToBLE();
}

function resize_on() {

}

