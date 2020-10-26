var ros;
var upper_limit_sub;
var lower_limit_sub;
var gui_cmd_pub;

var set_zero_torque_srv;
var set_lower_limit_srv;
var set_upper_limit_srv;
var reset_lower_limit_srv;
var reset_upper_limit_srv;
var enable_lower_limit_srv;
var enable_upper_limit_srv;
var set_zero_torque_srv;

var upper_limit;
var lower_limit;
var speed_setpoint;
var speed_cmd = 0.0;

var mainTimer;

var MAX_SPEED = 36.0;

var enable_loop = true;
function stopLoop() {
  enable_loop = false;
  $('#status_spinner_color').removeClass();
  $('#status_spinner_color').addClass('text-danger');
  $('#status_spinner').removeClass();
  $('#status_spinner').addClass('spinner-grow spinner-grow-sm');
}

function UpdatePage() {
  $('#upper_limit').val(upper_limit);
  $('#lower_limit').val(lower_limit);
}


function publishSpeed() {
  var msg = new ROSLIB.Message({
    joint1: parseFloat(speed_cmd),
    joint2: 0.0,
    joint3: 0.0,
    joint4: 0.0,
    joint5: 0.0,
    joint6: 0.0,
    joint7: 0.0
  });
  gui_cmd_pub.publish(msg);
}

function mainLoop() {
  if (enable_loop) {
    // console.log(speed_setpoint);
    // UpdatePage();
    publishSpeed();
  }
}

function SetLowerLimit() {
  console.log('SetLowerLimit');

  var request = new ROSLIB.ServiceRequest({});
  set_lower_limit_srv.callService(request, function (result) { });
}

function SetUpperLimit() {
  console.log('SetUpperLimit');

  var request = new ROSLIB.ServiceRequest({});
  set_upper_limit_srv.callService(request, function (result) { });
}

function ResetLowerLimit() {
  console.log('ResetLowerLimit');

  var request = new ROSLIB.ServiceRequest({});
  reset_lower_limit_srv.callService(request, function (result) { });
}

function ResetUpperLimit() {
  console.log('ResetUpperLimit');

  var request = new ROSLIB.ServiceRequest({});
  reset_upper_limit_srv.callService(request, function (result) { });
}

function EnableLowerLimit(value) {
  console.log('EnableLowerLimit');

  var request = new ROSLIB.ServiceRequest({
    data: value
  });
  enable_lower_limit_srv.callService(request, function (result) { });
}

function EnableUpperLimit(value) {
  console.log('EnableUpperLimit');

  var request = new ROSLIB.ServiceRequest({
    data: value
  });
  enable_upper_limit_srv.callService(request, function (result) { });
}

function SetZeroTorque(value) {
  console.log('SetZeroTorque');

  var request = new ROSLIB.ServiceRequest({});
  set_zero_torque_srv.callService(request, function (result) { });
}

function GoUp() {
  speed_cmd = speed_setpoint;
}

function GoDown() {
  speed_cmd = -speed_setpoint;
}

function StopMotion() {
  speed_cmd = 0.0;
}

window.onload = function () {
  speed_cmd = 0.0;

  var robot_IP = document.domain;
  if (robot_IP == "") {
    robot_IP = "localhost";
  }
  var ros_master_uri = "ws://" + robot_IP + ":9090";

  // Connecting to ROS
  // -----------------
  ros = new ROSLIB.Ros({
    url: ros_master_uri
  });

  ros.on('connection', function () {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
    stopLoop();
  });

  ros.on('close', function () {
    console.log('Connection to websocket server closed.');
    stopLoop();
  });

  // Setup topic subscribers
  // ----------------------
  upper_limit_sub = new ROSLIB.Topic({
    ros: ros,
    name: '/orthopus_addon/upper_limit',
    messageType: 'kinova_msgs/JointAngles'
  });
  upper_limit_sub.subscribe(function (message) {
    console.debug('Received message on ' + upper_limit_sub.name + ': ' + message.joint1);
    upper_limit = message.joint1;
    UpdatePage();
  });

  lower_limit_sub = new ROSLIB.Topic({
    ros: ros,
    name: '/orthopus_addon/lower_limit',
    messageType: 'kinova_msgs/JointAngles'
  });
  lower_limit_sub.subscribe(function (message) {
    console.debug('Received message on ' + lower_limit_sub.name + ': ' + message.joint1);
    lower_limit = message.joint1;
    UpdatePage();
  });

  // Setup topic publishers 
  // ----------------------
  gui_cmd_pub = new ROSLIB.Topic({
    ros: ros,
    name: '/orthopus_addon/gui_cmd',
    messageType: 'kinova_msgs/JointAngles'
  });

  // Setup service 
  // ----------------------
  set_lower_limit_srv = new ROSLIB.Service({
    ros: ros,
    name: '/orthopus_addon/set_lower_limit',
    serviceType: 'std_srvs/Empty'
  });

  set_upper_limit_srv = new ROSLIB.Service({
    ros: ros,
    name: '/orthopus_addon/set_upper_limit',
    serviceType: 'std_srvs/Empty'
  });

  reset_lower_limit_srv = new ROSLIB.Service({
    ros: ros,
    name: '/orthopus_addon/reset_lower_limit',
    serviceType: 'std_srvs/Empty'
  });

  reset_upper_limit_srv = new ROSLIB.Service({
    ros: ros,
    name: '/orthopus_addon/reset_upper_limit',
    serviceType: 'std_srvs/Empty'
  });

  enable_lower_limit_srv = new ROSLIB.Service({
    ros: ros,
    name: '/orthopus_addon/enable_lower_limit',
    serviceType: 'std_srvs/SetBool'
  });

  enable_upper_limit_srv = new ROSLIB.Service({
    ros: ros,
    name: '/orthopus_addon/enable_upper_limit',
    serviceType: 'std_srvs/SetBool'
  });

  set_zero_torque_srv = new ROSLIB.Service({
    ros: ros,
    name: '/j2n6s300_driver/in/set_zero_torques',
    serviceType: 'kinova_msgs/ ZeroTorques'
  });


  $('#speed_slider').attr('max', MAX_SPEED);
  $('#speed_slider').attr('value', (MAX_SPEED / 2.0));
  $('#speed_slider').val(MAX_SPEED / 2.0);
  rangeSlider();
  speed_setpoint = $('#speed_slider').val();

  $('#enable_upperlim')[0].checked = true;
  $('#enable_lowerlim')[0].checked = true;
  EnableUpperLimit(true);
  EnableLowerLimit(true);

  // Setup main loop timer
  // ----------------------
  mainTimer = setInterval(function () { mainLoop(); }, 100);
}

var rangeSlider = function () {
  var slider = $('.range-slider'),
    range = $('.range-slider__range'),
    value = $('.range-slider__value');

  slider.each(function () {
    value.each(function () {
      var value = $(this).prev().attr('value');
      $(this).html(value);
    });

    range.on('input', function () {
      $(this).next(value).html(this.value);
    });
  });
};

$(function () {

  $('#set_lowerlim')
    .click(function () {
      SetLowerLimit();
    });

  $('#set_upperlim')
    .click(function () {
      SetUpperLimit();
    });

  $('#reset_lowerlim')
    .click(function () {
      ResetLowerLimit();
    });

  $('#reset_upperlim')
    .click(function () {
      ResetUpperLimit();
    });

  $('#up_btn')
    .mousedown(function () {
      GoUp();
    })
    .mouseup(function () {
      StopMotion();
    });

  $('#down_btn')
    .mousedown(function () {
      GoDown();
    })
    .mouseup(function () {
      StopMotion();
    });

  $('#set_zero_torque')
    .click(function () {
      SetZeroTorque();
    });

  $('#enable_lowerlim')
    .change(function () {
      var state = $('#enable_lowerlim')[0].checked;
      EnableLowerLimit(state);
    });

  $('#enable_upperlim')
    .change(function () {
      var state = $('#enable_upperlim')[0].checked;
      EnableUpperLimit(state);
    });

  /* Configure speed slider calback */
  $('#speed_slider')
    .on('input', function () {
      speed_setpoint = this.value;
    });

});


