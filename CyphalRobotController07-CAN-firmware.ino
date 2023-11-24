/**
 * Default firmware for the CyphalPicoBase-CAN (https://github.com/generationmake/CyphalPicoBase-CAN)
 *
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/CyphalPicoBase-CAN-firmware/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <SPI.h>
#include <Wire.h>

#include <107-Arduino-Cyphal.h>
#include <107-Arduino-Cyphal-Support.h>

#include <107-Arduino-MCP2515.h>
#include <107-Arduino-littlefs.h>
#include <107-Arduino-24LCxx.hpp>
#include <INA226_WE.h>
#include <ADS1115_WE.h>
#include "ifx007t.h"
#include "pio_encoder.h"


#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
#define DBG_ENABLE_DEBUG
//#define DBG_ENABLE_VERBOSE
#include <107-Arduino-Debug.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t const EEPROM_I2C_DEV_ADDR = 0x50;

static int const MCP2515_CS_PIN     = 17;
static int const MCP2515_INT_PIN    = 20;
static int const ENCODER0_A         = 2;
static int const ENCODER0_B         = 3;
static int const ENCODER1_A         = 14;
static int const ENCODER1_B         = 15;
static int const MOTOR0_1           = 9;
static int const MOTOR0_2           = 8;
static int const MOTOR1_1           = 7;
static int const MOTOR1_2           = 6;
static int const MOTOR1_EN          = 10;
static int const MOTOR0_EN          = 11;
static int const EM_STOP_PIN        = 12;
static int const OUTPUT_0_PIN       = 21; /* GP21 */
static int const OUTPUT_1_PIN       = 22; /* GP22 */
static int const ANALOG_INPUT_0_PIN = 26;
static int const ANALOG_INPUT_1_PIN = 27;
static int const ANALOG_INPUT_2_PIN = 28;

static SPISettings const MCP2515x_SPI_SETTING{10*1000*1000UL, MSBFIRST, SPI_MODE0};

static uint16_t const UPDATE_PERIOD_HEARTBEAT_ms = 1000;

static uint32_t const WATCHDOG_DELAY_ms = 1000;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame);
ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

Ifx007t mot0;
Ifx007t mot1;
PioEncoder encoder0(ENCODER0_A);
PioEncoder encoder1(ENCODER1_A);
INA226_WE ina226 = INA226_WE();
ADS1115_WE ads1115 = ADS1115_WE();

DEBUG_INSTANCE(80, Serial);

ArduinoMCP2515 mcp2515([]() { digitalWrite(MCP2515_CS_PIN, LOW); },
                       []() { digitalWrite(MCP2515_CS_PIN, HIGH); },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr,
                       [](MCP2515::EFLG const err_flag) { DBG_ERROR("MCP2515::OnError, error code = \"%s\"", MCP2515::toStr(err_flag)); },
                       [](MCP2515::EFLG const err_flag) { DBG_ERROR("MCP2515::OnWarning, warning code = \"%s\"", MCP2515::toStr(err_flag)); });

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap;
cyphal::Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); });

cyphal::Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>(1*1000*1000UL /* = 1 sec in usecs. */);
cyphal::Publisher<uavcan::primitive::scalar::Real32_1_0> internal_temperature_pub;
cyphal::Publisher<uavcan::primitive::scalar::Real32_1_0> input_voltage_pub;
cyphal::Publisher<uavcan::primitive::scalar::Real32_1_0> input_current_pub;
cyphal::Publisher<uavcan::primitive::scalar::Real32_1_0> input_power_pub;
cyphal::Publisher<uavcan::primitive::scalar::Real32_1_0> input_current_total_pub;
cyphal::Publisher<uavcan::primitive::scalar::Real32_1_0> input_power_total_pub;
cyphal::Publisher<uavcan::primitive::scalar::Bit_1_0> em_stop_pub;
cyphal::Publisher<uavcan::primitive::scalar::Integer16_1_0> analog_input_0_pub;
cyphal::Publisher<uavcan::primitive::scalar::Integer16_1_0> analog_input_1_pub;
cyphal::Publisher<uavcan::primitive::scalar::Integer16_1_0> analog_input_2_pub;
cyphal::Publisher<uavcan::primitive::scalar::Integer16_1_0> motor0_current_pub;
cyphal::Publisher<uavcan::primitive::scalar::Integer16_1_0> motor1_current_pub;
cyphal::Publisher<uavcan::primitive::scalar::Integer16_1_0> motor0_bemf_pub;
cyphal::Publisher<uavcan::primitive::scalar::Integer16_1_0> motor1_bemf_pub;
cyphal::Publisher<uavcan::primitive::scalar::Integer32_1_0> encoder0_pub;
cyphal::Publisher<uavcan::primitive::scalar::Integer32_1_0> encoder1_pub;

cyphal::Subscription output_0_subscription, output_1_subscription;
cyphal::Subscription motor_0_subscription, motor_1_subscription;

cyphal::ServiceServer execute_command_srv = node_hdl.create_service_server<ExecuteCommand::Request_1_1, ExecuteCommand::Response_1_1>(2*1000*1000UL, onExecuteCommand_1_1_Request_Received);

/* LITTLEFS/EEPROM ********************************************************************/

static EEPROM_24LCxx eeprom(EEPROM_24LCxx_Type::LC64,
                            EEPROM_I2C_DEV_ADDR,
                            [](size_t const dev_addr) { Wire.beginTransmission(dev_addr); },
                            [](uint8_t const data) { Wire.write(data); },
                            []() { return Wire.endTransmission(); },
                            [](uint8_t const dev_addr, size_t const len) -> size_t { return Wire.requestFrom(dev_addr, len); },
                            []() { return Wire.available(); },
                            []() { return Wire.read(); });

static littlefs::FilesystemConfig filesystem_config
  (
    +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) -> int
    {
      eeprom.read_page((block * c->block_size) + off, (uint8_t *)buffer, size);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) -> int
    {
      eeprom.write_page((block * c->block_size) + off, (uint8_t const *)buffer, size);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c, lfs_block_t block) -> int
    {
      for(size_t off = 0; off < c->block_size; off += eeprom.page_size())
        eeprom.fill_page((block * c->block_size) + off, 0xFF);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c) -> int
    {
      return LFS_ERR_OK;
    },
    eeprom.page_size(),
    eeprom.page_size(),
    (eeprom.page_size() * 4), /* littlefs demands (erase) block size to exceed read/prog size. */
    eeprom.device_size() / (eeprom.page_size() * 4),
    500,
    eeprom.page_size(),
    eeprom.page_size()
  );
static littlefs::Filesystem filesystem(filesystem_config);

#if __GNUC__ >= 11
cyphal::support::platform::storage::littlefs::KeyValueStorage kv_storage(filesystem);
#endif /* __GNUC__ >= 11 */

/* REGISTER ***************************************************************************/

static uint16_t     node_id                      = std::numeric_limits<uint16_t>::max();
static CanardPortID port_id_internal_temperature = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_input_voltage        = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_input_current        = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_input_power          = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_input_current_total  = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_input_power_total    = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_em_stop              = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_output0              = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_output1              = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_motor0               = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_motor1               = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_analog_input0        = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_analog_input1        = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_analog_input2        = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_motor0_current       = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_motor1_current       = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_motor0_bemf          = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_motor1_bemf          = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_encoder0             = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_encoder1             = std::numeric_limits<CanardPortID>::max();

static uint16_t update_period_ms_internaltemperature = 10*1000;
static uint16_t update_period_ms_input_voltage       =  1*1000;
static uint16_t update_period_ms_input_current       =  1*1000;
static uint16_t update_period_ms_input_power         =  1*1000;
static uint16_t update_period_ms_input_current_total =  5*1000;
static uint16_t update_period_ms_input_power_total   =  5*1000;
static uint16_t update_period_ms_em_stop             =     500;
static uint16_t update_period_ms_analoginput0        =     500;
static uint16_t update_period_ms_analoginput1        =     500;
static uint16_t update_period_ms_analoginput2        =     500;
static uint16_t update_period_ms_motor0_current      =    1000;
static uint16_t update_period_ms_motor1_current      =    1000;
static uint16_t update_period_ms_motor0_bemf         =    1000;
static uint16_t update_period_ms_motor1_bemf         =    1000;
static uint16_t update_period_ms_encoder0            =    1000;
static uint16_t update_period_ms_encoder1            =    1000;
static uint16_t timeout_ms_motor0                    =    1000;
static uint16_t timeout_ms_motor1                    =    1000;
static bool reverse_motor_0                          = false;
static bool reverse_motor_1                          = false;
static unsigned long prev_motor0_update              = 0;
static unsigned long prev_motor1_update              = 0;

static std::string node_description{"CyphalRobotController07/CAN"};

#if __GNUC__ >= 11

const auto node_registry = node_hdl.create_registry();

const auto reg_rw_cyphal_node_id                            = node_registry->expose("cyphal.node.id",                           {true}, node_id);
const auto reg_rw_cyphal_node_description                   = node_registry->expose("cyphal.node.description",                  {true}, node_description);
const auto reg_rw_cyphal_pub_internaltemperature_id         = node_registry->expose("cyphal.pub.internaltemperature.id",        {true}, port_id_internal_temperature);
const auto reg_ro_cyphal_pub_internaltemperature_type       = node_registry->route ("cyphal.pub.internaltemperature.type",      {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_rw_cyphal_pub_input_voltage_id               = node_registry->expose("cyphal.pub.inputvoltage.id",               {true}, port_id_input_voltage);
const auto reg_ro_cyphal_pub_input_voltage_type             = node_registry->route ("cyphal.pub.inputvoltage.type",             {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_rw_cyphal_pub_input_current_id               = node_registry->expose("cyphal.pub.inputcurrent.id",               {true}, port_id_input_current);
const auto reg_ro_cyphal_pub_input_current_type             = node_registry->route ("cyphal.pub.inputcurrent.type",             {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_rw_cyphal_pub_input_power_id                 = node_registry->expose("cyphal.pub.inputpower.id",                 {true}, port_id_input_power);
const auto reg_ro_cyphal_pub_input_power_type               = node_registry->route ("cyphal.pub.inputpower.type",               {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_rw_cyphal_pub_input_current_total_id         = node_registry->expose("cyphal.pub.inputcurrenttotal.id",          {true}, port_id_input_current_total);
const auto reg_ro_cyphal_pub_input_current_total_type       = node_registry->route ("cyphal.pub.inputcurrenttotal.type",        {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_rw_cyphal_pub_input_power_total_id           = node_registry->expose("cyphal.pub.inputpowertotal.id",            {true}, port_id_input_power_total);
const auto reg_ro_cyphal_pub_input_power_total_type         = node_registry->route ("cyphal.pub.inputpowertotal.type",          {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_rw_cyphal_pub_em_stop_id                     = node_registry->expose("cyphal.pub.em_stop.id",                    {true}, port_id_em_stop);
const auto reg_ro_cyphal_pub_em_stop_type                   = node_registry->route ("cyphal.pub.em_stop.type",                  {true}, []() { return "uavcan.primitive.scalar.Bit.1.0"; });
const auto reg_rw_cyphal_pub_analoginput0_id                = node_registry->expose("cyphal.pub.analoginput0.id",               {true}, port_id_analog_input0);
const auto reg_ro_cyphal_pub_analoginput0_type              = node_registry->route ("cyphal.pub.analoginput0.type",             {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_pub_analoginput1_id                = node_registry->expose("cyphal.pub.analoginput1.id",               {true}, port_id_analog_input1);
const auto reg_ro_cyphal_pub_analoginput1_type              = node_registry->route ("cyphal.pub.analoginput1.type",             {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_pub_analoginput2_id                = node_registry->expose("cyphal.pub.analoginput2.id",               {true}, port_id_analog_input2);
const auto reg_ro_cyphal_pub_analoginput2_type              = node_registry->route ("cyphal.pub.analoginput2.type",             {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_pub_motor0_current_id              = node_registry->expose("cyphal.pub.motor0current.id",              {true}, port_id_motor0_current);
const auto reg_ro_cyphal_pub_motor0_current_type            = node_registry->route ("cyphal.pub.motor0current.type",            {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_pub_motor1_current_id              = node_registry->expose("cyphal.pub.motor1current.id",              {true}, port_id_motor1_current);
const auto reg_ro_cyphal_pub_motor1_current_type            = node_registry->route ("cyphal.pub.motor1current.type",            {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_pub_motor0_bemf_id                 = node_registry->expose("cyphal.pub.motor0bemf.id",                 {true}, port_id_motor0_bemf);
const auto reg_ro_cyphal_pub_motor0_bemf_type               = node_registry->route ("cyphal.pub.motor0bemf.type",               {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_pub_motor1_bemf_id                 = node_registry->expose("cyphal.pub.motor1bemf.id",                 {true}, port_id_motor1_bemf);
const auto reg_ro_cyphal_pub_motor1_bemf_type               = node_registry->route ("cyphal.pub.motor1bemf.type",               {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_pub_encoder0_id                    = node_registry->expose("cyphal.pub.encoder0.id",                   {true}, port_id_encoder0);
const auto reg_ro_cyphal_pub_encoder0_type                  = node_registry->route ("cyphal.pub.encoder0.type",                 {true}, []() { return "uavcan.primitive.scalar.Integer32.1.0"; });
const auto reg_rw_cyphal_pub_encoder1_id                    = node_registry->expose("cyphal.pub.encoder1.id",                   {true}, port_id_encoder1);
const auto reg_ro_cyphal_pub_encoder1_type                  = node_registry->route ("cyphal.pub.encoder1.type",                 {true}, []() { return "uavcan.primitive.scalar.Integer32.1.0"; });
const auto reg_rw_cyphal_sub_output0_id                     = node_registry->expose("cyphal.sub.output0.id",                    {true}, port_id_output0);
const auto reg_ro_cyphal_sub_output0_type                   = node_registry->route ("cyphal.sub.output0.type",                  {true}, []() { return "uavcan.primitive.scalar.Bit.1.0"; });
const auto reg_rw_cyphal_sub_output1_id                     = node_registry->expose("cyphal.sub.output1.id",                    {true}, port_id_output1);
const auto reg_ro_cyphal_sub_output1_type                   = node_registry->route ("cyphal.sub.output1.type",                  {true}, []() { return "uavcan.primitive.scalar.Bit.1.0"; });
const auto reg_rw_cyphal_sub_motor0_id                      = node_registry->expose("cyphal.sub.motor0.id",                     {true}, port_id_motor0);
const auto reg_ro_cyphal_sub_motor0_type                    = node_registry->route ("cyphal.sub.motor0.type",                   {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_sub_motor1_id                      = node_registry->expose("cyphal.sub.motor1.id",                     {true}, port_id_motor1);
const auto reg_ro_cyphal_sub_motor1_type                    = node_registry->route ("cyphal.sub.motor1.type",                   {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_crc07_update_period_ms_internaltemperature = node_registry->expose("crc07.update_period_ms.internaltemperature", {true}, update_period_ms_internaltemperature);
const auto reg_rw_crc07_update_period_ms_input_voltage       = node_registry->expose("crc07.update_period_ms.inputvoltage",        {true}, update_period_ms_input_voltage);
const auto reg_rw_crc07_update_period_ms_input_current       = node_registry->expose("crc07.update_period_ms.inputcurrent",        {true}, update_period_ms_input_current);
const auto reg_rw_crc07_update_period_ms_input_power         = node_registry->expose("crc07.update_period_ms.inputpower",          {true}, update_period_ms_input_power);
const auto reg_rw_crc07_update_period_ms_input_current_total = node_registry->expose("crc07.update_period_ms.inputcurrenttotal",   {true}, update_period_ms_input_current_total);
const auto reg_rw_crc07_update_period_ms_input_power_total   = node_registry->expose("crc07.update_period_ms.inputpowertotal",     {true}, update_period_ms_input_power_total);
const auto reg_rw_crc07_update_period_ms_em_stop             = node_registry->expose("crc07.update_period_ms.em_stop",             {true}, update_period_ms_em_stop);
const auto reg_rw_crc07_update_period_ms_analoginput0        = node_registry->expose("crc07.update_period_ms.analoginput0",        {true}, update_period_ms_analoginput0);
const auto reg_rw_crc07_update_period_ms_analoginput1        = node_registry->expose("crc07.update_period_ms.analoginput1",        {true}, update_period_ms_analoginput1);
const auto reg_rw_crc07_update_period_ms_analoginput2        = node_registry->expose("crc07.update_period_ms.analoginput2",        {true}, update_period_ms_analoginput2);
const auto reg_rw_crc07_update_period_ms_motor0_current      = node_registry->expose("crc07.update_period_ms.motor0current",       {true}, update_period_ms_motor0_current);
const auto reg_rw_crc07_update_period_ms_motor1_current      = node_registry->expose("crc07.update_period_ms.motor1current",       {true}, update_period_ms_motor1_current);
const auto reg_rw_crc07_update_period_ms_motor0_bemf         = node_registry->expose("crc07.update_period_ms.motor0bemf",          {true}, update_period_ms_motor0_bemf);
const auto reg_rw_crc07_update_period_ms_motor1_bemf         = node_registry->expose("crc07.update_period_ms.motor1bemf",          {true}, update_period_ms_motor1_bemf);
const auto reg_rw_crc07_update_period_ms_encoder0            = node_registry->expose("crc07.update_period_ms.encoder0",            {true}, update_period_ms_encoder0);
const auto reg_rw_crc07_update_period_ms_encoder1            = node_registry->expose("crc07.update_period_ms.encoder1",            {true}, update_period_ms_encoder1);
const auto reg_rw_crc07_timeout_ms_motor0                    = node_registry->expose("crc07.timeout_ms.motor0",                    {true}, timeout_ms_motor0);
const auto reg_rw_crc07_timeout_ms_motor1                    = node_registry->expose("crc07.timeout_ms.motor1",                    {true}, timeout_ms_motor1);
const auto reg_rw_crc07_reverse_motor0                       = node_registry->expose("crc07.motor_0.reverse",                      {true}, reverse_motor_0);
const auto reg_rw_crc07_reverse_motor1                       = node_registry->expose("crc07.motor_1.reverse",                      {true}, reverse_motor_1);

#endif /* __GNUC__ >= 11 */

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  // while(!Serial) { } /* only for debug */
  delay(1000);

  Debug.prettyPrintOn(); /* Enable pretty printing on a shell. */

  /* LITTLEFS/EEPROM ********************************************************************/
  Wire.begin();
  Wire.setClock(400*1000UL); /* Set fast mode. */

  if (!eeprom.isConnected()) {
    DBG_ERROR("Connecting to EEPROM failed.");
    return;
  }
  Serial.println(eeprom);

  if (auto const err_mount = filesystem.mount(); err_mount.has_value()) {
    DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount.value()));
    (void)filesystem.format();
  }

  if (auto const err_mount = filesystem.mount(); err_mount.has_value()) {
    DBG_ERROR("Mounting failed again with error code %d", static_cast<int>(err_mount.value()));
    return;
  }

#if __GNUC__ >= 11
  auto const rc_load = cyphal::support::load(kv_storage, *node_registry);
  if (rc_load.has_value()) {
    DBG_ERROR("cyphal::support::load failed with %d", static_cast<int>(rc_load.value()));
    return;
  }
#endif /* __GNUC__ >= 11 */

  (void)filesystem.unmount();

  /* If the node ID contained in the register points to an undefined
   * node ID, assign node ID 0 to this node.
   */
  if (node_id > CANARD_NODE_ID_MAX)
    node_id = 0;
  node_hdl.setNodeId(static_cast<CanardNodeID>(node_id));

  if (port_id_internal_temperature != std::numeric_limits<CanardPortID>::max())
    internal_temperature_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Real32_1_0>(port_id_internal_temperature, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_input_voltage != std::numeric_limits<CanardPortID>::max())
    input_voltage_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Real32_1_0>(port_id_input_voltage, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_input_current != std::numeric_limits<CanardPortID>::max())
    input_current_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Real32_1_0>(port_id_input_current, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_input_power != std::numeric_limits<CanardPortID>::max())
    input_power_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Real32_1_0>(port_id_input_power, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_input_current_total != std::numeric_limits<CanardPortID>::max())
    input_current_total_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Real32_1_0>(port_id_input_current_total, 5*1000*1000UL /* = 5 sec in usecs. */);
  if (port_id_input_power_total != std::numeric_limits<CanardPortID>::max())
    input_power_total_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Real32_1_0>(port_id_input_power_total, 5*1000*1000UL /* = 5 sec in usecs. */);

  if (port_id_output0 != std::numeric_limits<CanardPortID>::max())
    output_0_subscription = node_hdl.create_subscription<uavcan::primitive::scalar::Bit_1_0>(
      port_id_output0,
      [](uavcan::primitive::scalar::Bit_1_0 const & msg)
      {
        if(msg.value)
          digitalWrite(OUTPUT_0_PIN, HIGH);
        else
          digitalWrite(OUTPUT_0_PIN, LOW);
      });

  if (port_id_output1 != std::numeric_limits<CanardPortID>::max())
    output_1_subscription = node_hdl.create_subscription<uavcan::primitive::scalar::Bit_1_0>(
      port_id_output1,
      [](uavcan::primitive::scalar::Bit_1_0 const & msg)
      {
        if(msg.value)
          digitalWrite(OUTPUT_1_PIN, HIGH);
        else
          digitalWrite(OUTPUT_1_PIN, LOW);
      });

  if (port_id_motor0 != std::numeric_limits<CanardPortID>::max())
    motor_0_subscription = node_hdl.create_subscription<uavcan::primitive::scalar::Integer16_1_0>(
      port_id_motor0,
      [](uavcan::primitive::scalar::Integer16_1_0 const & msg)
      {
        if (reverse_motor_0)
          mot0.pwm(-1 * msg.value);
        else
          mot0.pwm(msg.value);

        prev_motor0_update = millis();
      });

  if (port_id_motor1 != std::numeric_limits<CanardPortID>::max())
    motor_1_subscription = node_hdl.create_subscription<uavcan::primitive::scalar::Integer16_1_0>(
      port_id_motor1,
      [](uavcan::primitive::scalar::Integer16_1_0 const & msg)
      {
        if (reverse_motor_1)
          mot1.pwm(-1 * msg.value);
        else
          mot1.pwm(msg.value);

        prev_motor1_update=millis();
      });

  if (port_id_em_stop != std::numeric_limits<CanardPortID>::max())
    em_stop_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Bit_1_0>(port_id_em_stop, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_analog_input0 != std::numeric_limits<CanardPortID>::max())
    analog_input_0_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(port_id_analog_input0, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_analog_input1 != std::numeric_limits<CanardPortID>::max())
    analog_input_1_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(port_id_analog_input1, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_analog_input2 != std::numeric_limits<CanardPortID>::max())
    analog_input_2_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(port_id_analog_input2, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_motor0_current != std::numeric_limits<CanardPortID>::max())
    motor0_current_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(port_id_motor0_current, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_motor1_current != std::numeric_limits<CanardPortID>::max())
    motor1_current_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(port_id_motor1_current, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_motor0_bemf != std::numeric_limits<CanardPortID>::max())
    motor0_bemf_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(port_id_motor0_bemf, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_motor1_bemf != std::numeric_limits<CanardPortID>::max())
    motor1_bemf_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(port_id_motor1_bemf, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_encoder0 != std::numeric_limits<CanardPortID>::max())
    encoder0_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer32_1_0>(port_id_encoder0, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_encoder1 != std::numeric_limits<CanardPortID>::max())
    encoder1_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer32_1_0>(port_id_encoder1, 1*1000*1000UL /* = 1 sec in usecs. */);
  /* set factory settings */
  if(update_period_ms_internaltemperature==0xFFFF) update_period_ms_internaltemperature=10*1000;
  if(update_period_ms_input_voltage==0xFFFF)       update_period_ms_input_voltage=1*1000;
  if(update_period_ms_input_current==0xFFFF)       update_period_ms_input_current=1*1000;
  if(update_period_ms_input_power==0xFFFF)         update_period_ms_input_power=1*1000;
  if(update_period_ms_input_current_total==0xFFFF) update_period_ms_input_current_total=5*1000;
  if(update_period_ms_input_power_total==0xFFFF)   update_period_ms_input_power_total=5*1000;
  if(update_period_ms_em_stop==0xFFFF)             update_period_ms_em_stop=500;
  if(update_period_ms_analoginput0==0xFFFF)        update_period_ms_analoginput0=500;
  if(update_period_ms_analoginput1==0xFFFF)        update_period_ms_analoginput1=500;
  if(update_period_ms_analoginput2==0xFFFF)        update_period_ms_analoginput2=500;
  if(update_period_ms_motor0_current==0xFFFF)      update_period_ms_motor0_current=1000;
  if(update_period_ms_motor1_current==0xFFFF)      update_period_ms_motor1_current=1000;
  if(update_period_ms_motor0_bemf==0xFFFF)         update_period_ms_motor0_bemf=1000;
  if(update_period_ms_motor1_bemf==0xFFFF)         update_period_ms_motor1_bemf=1000;
  if(update_period_ms_encoder0==0xFFFF)            update_period_ms_encoder0=1000;
  if(update_period_ms_encoder1==0xFFFF)            update_period_ms_encoder1=1000;

  /* NODE INFO **************************************************************************/
  static const auto node_info = node_hdl.create_node_info
  (
    /* cyphal.node.Version.1.0 protocol_version */
    1, 0,
    /* cyphal.node.Version.1.0 hardware_version */
    1, 0,
    /* cyphal.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
#ifdef CYPHAL_NODE_INFO_GIT_VERSION
    CYPHAL_NODE_INFO_GIT_VERSION,
#else
    0,
#endif
    /* saturated uint8[16] unique_id */
    cyphal::support::UniqueId::instance().value(),
    /* saturated uint8[<=50] name */
    "107-systems.cyphal-robot-controller-07"
  );

  /* Setup LED pins and initialize */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(EM_STOP_PIN, INPUT_PULLDOWN);

  /* Setup OUT0/OUT1. */
  pinMode(OUTPUT_0_PIN, OUTPUT);
  pinMode(OUTPUT_1_PIN, OUTPUT);
  digitalWrite(OUTPUT_0_PIN, LOW);
  digitalWrite(OUTPUT_1_PIN, LOW);

  /* Setup SPI access */
  SPI.begin();
  SPI.beginTransaction(MCP2515x_SPI_SETTING);
  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  pinMode(MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MCP2515_CS_PIN, HIGH);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);

  /* Leave configuration and enable MCP2515. */
  mcp2515.setNormalMode();

  /* configure motor pwm output */
  analogWriteFreq(4000);
  mot0.begin(MOTOR0_1,MOTOR0_2,MOTOR0_EN);
  mot1.begin(MOTOR1_1,MOTOR1_2,MOTOR1_EN);

  /* configure encoder input */
  encoder0.begin();
  encoder1.begin();

  /* configure INA226, current sensor, set conversion time and average to get a value every two seconds */
  ina226.init();
  ina226.setResistorRange(0.020,4.0); // choose resistor 20 mOhm and gain range up to 4 A
  ina226.setAverage(AVERAGE_512);
  ina226.setConversionTime(CONV_TIME_4156);

  /* configure ADS1115 */
  ads1115.init();
  ads1115.setVoltageRange_mV(ADS1115_RANGE_6144); //comment line/change parameter to change range
  ads1115.setCompareChannels(ADS1115_COMP_0_GND); //comment line/change parameter to change channel
  ads1115.setMeasureMode(ADS1115_CONTINUOUS); //comment line/change parameter to change mode

  /* Enable watchdog. */
  rp2040.wdt_begin(WATCHDOG_DELAY_ms);
  rp2040.wdt_reset();

  DBG_INFO("Init complete.");
}

void loop()
{
  /* Deal with all pending events of the MCP2515 -
   * signaled by the INT pin being driven LOW.
   */
  while(digitalRead(MCP2515_INT_PIN) == LOW)
    mcp2515.onExternalEventHandler();

  /* Process all pending Cyphal actions.
   */
  node_hdl.spinSome();

  /* Publish all the gathered data, although at various
   * different intervals.
   */
  static unsigned long prev_heartbeat = 0;
  static unsigned long prev_internal_temperature = 0;
  static unsigned long prev_em_stop = 0;
  static unsigned long prev_analog_input0 = 0;
  static unsigned long prev_analog_input1 = 0;
  static unsigned long prev_analog_input2 = 0;
  static unsigned long prev_motor0_current = 0;
  static unsigned long prev_motor1_current = 0;
  static unsigned long prev_motor0_bemf = 0;
  static unsigned long prev_motor1_bemf = 0;
  static unsigned long prev_encoder0 = 0;
  static unsigned long prev_encoder1 = 0;
  static unsigned long prev_input_voltage = 0;
  static unsigned long prev_input_current = 0;
  static unsigned long prev_input_power = 0;
  static unsigned long prev_input_current_total = 0;
  static unsigned long prev_input_power_total = 0;

  static unsigned long prev_ina226 = 0;
  static float ina226_busVoltage_V = 0.0;
  static float ina226_current_mA = 0.0;
  static float ina226_power_mW = 0.0;
  static float ina226_current_total_mAh = 0.0;
  static float ina226_power_total_mWh = 0.0;

  static unsigned long prev_ads1115 = 0;
  static int ads1115_data0 = 0;
  static int ads1115_data1 = 0;
  static int ads1115_data2 = 0;
  static int ads1115_data3 = 0;

  unsigned long const now = millis();

  /* get ADS1115 data ever 100 ms */
  if((now - prev_ads1115) > 100)
  {
    prev_ads1115 = now;
    static int ads1115_count = 0;
    ads1115_count ++;
    if(ads1115_count >= 4) ads1115_count=0;

    if(ads1115_count == 0)
    {
      ads1115_data0 = ads1115.getRawResult();
      ads1115.setCompareChannels_nonblock(ADS1115_COMP_1_GND);
    }
    else if(ads1115_count == 1)
    {
      ads1115_data1 = ads1115.getRawResult();
      ads1115.setCompareChannels_nonblock(ADS1115_COMP_2_GND);
    }
    else if(ads1115_count == 2)
    {
      ads1115_data2 = ads1115.getRawResult();
      ads1115.setCompareChannels_nonblock(ADS1115_COMP_3_GND);
    }
    else if(ads1115_count == 3)
    {
      ads1115_data3 = ads1115.getRawResult();
      ads1115.setCompareChannels_nonblock(ADS1115_COMP_0_GND);
    }
  }
  /* get INA226 data once/second */
  if((now - prev_ina226) > 1000)
  {
    prev_ina226 = now;

    ina226.readAndClearFlags();
    ina226_busVoltage_V = ina226.getBusVoltage_V();
    ina226_current_mA = ina226.getCurrent_mA();
    ina226_power_mW = ina226.getBusPower();
    ina226_current_total_mAh += ina226_current_mA/3600.0;
    ina226_power_total_mWh += ina226_power_mW/3600.0;
  }
  /* check motor timeout */
  if(timeout_ms_motor0<0xFFFF)
  {
    if((now - prev_motor0_update) > timeout_ms_motor0)
    {
      mot0.pwm(0);
    }
  }
  if(timeout_ms_motor1<0xFFFF)
  {
    if((now - prev_motor1_update) > timeout_ms_motor1)
    {
      mot1.pwm(0);
    }
  }
  /* Publish the heartbeat once/second */
  if((now - prev_heartbeat) > UPDATE_PERIOD_HEARTBEAT_ms)
  {
    prev_heartbeat = now;

    Heartbeat_1_0 msg;

    msg.uptime = millis() / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);
  }

  if((now - prev_internal_temperature) > (update_period_ms_internaltemperature))
  {
    float const temperature = analogReadTemp();
    Serial.print("Temperature: ");
    Serial.println(temperature);

    uavcan::primitive::scalar::Real32_1_0 uavcan_internal_temperature;
    uavcan_internal_temperature.value = temperature;
    if(internal_temperature_pub) internal_temperature_pub->publish(uavcan_internal_temperature);

    prev_internal_temperature = now;
  }
  if((now - prev_em_stop) > update_period_ms_em_stop)
  {
    uavcan::primitive::scalar::Bit_1_0 uavcan_em_stop;
    uavcan_em_stop.value = digitalRead(EM_STOP_PIN);
    if(em_stop_pub) em_stop_pub->publish(uavcan_em_stop);

    prev_em_stop = now;
  }

  if((now - prev_analog_input0) > update_period_ms_analoginput0)
  {
    uavcan::primitive::scalar::Integer16_1_0 uavcan_analog_input0;
    uavcan_analog_input0.value = analogRead(ANALOG_INPUT_0_PIN);
    if(analog_input_0_pub) analog_input_0_pub->publish(uavcan_analog_input0);

    prev_analog_input0 = now;
  }
  if((now - prev_analog_input1) > update_period_ms_analoginput1)
  {
    uavcan::primitive::scalar::Integer16_1_0 uavcan_analog_input1;
    uavcan_analog_input1.value = analogRead(ANALOG_INPUT_1_PIN);
    if(analog_input_1_pub) analog_input_1_pub->publish(uavcan_analog_input1);

    prev_analog_input1 = now;
  }
  if((now - prev_analog_input2) > update_period_ms_analoginput2)
  {
    uavcan::primitive::scalar::Integer16_1_0 uavcan_analog_input2;
    uavcan_analog_input2.value = analogRead(ANALOG_INPUT_2_PIN);
    if(analog_input_2_pub) analog_input_2_pub->publish(uavcan_analog_input2);

    prev_analog_input2 = now;
  }
  if((now - prev_input_voltage) > (update_period_ms_input_voltage))
  {
    uavcan::primitive::scalar::Real32_1_0 uavcan_input_voltage;
    uavcan_input_voltage.value = ina226_busVoltage_V;
    if(input_voltage_pub) input_voltage_pub->publish(uavcan_input_voltage);

    prev_input_voltage = now;
  }
  if((now - prev_input_current) > (update_period_ms_input_current))
  {
    uavcan::primitive::scalar::Real32_1_0 uavcan_input_current;
    uavcan_input_current.value = ina226_current_mA;
    if(input_current_pub) input_current_pub->publish(uavcan_input_current);

    prev_input_current = now;
  }
  if((now - prev_input_power) > (update_period_ms_input_power))
  {
    uavcan::primitive::scalar::Real32_1_0 uavcan_input_power;
    uavcan_input_power.value = ina226_power_mW;
    if(input_power_pub) input_power_pub->publish(uavcan_input_power);

    prev_input_power = now;
  }
  if((now - prev_input_current_total) > (update_period_ms_input_current_total))
  {
    uavcan::primitive::scalar::Real32_1_0 uavcan_input_current_total;
    uavcan_input_current_total.value = ina226_current_total_mAh;
    if(input_current_total_pub) input_current_total_pub->publish(uavcan_input_current_total);

    prev_input_current_total = now;
  }
  if((now - prev_input_power_total) > (update_period_ms_input_power_total))
  {
    uavcan::primitive::scalar::Real32_1_0 uavcan_input_power_total;
    uavcan_input_power_total.value = ina226_power_total_mWh;
    if(input_power_total_pub) input_power_total_pub->publish(uavcan_input_power_total);

    prev_input_power_total = now;
  }
  if((now - prev_motor0_current) > update_period_ms_motor0_current)
  {
    uavcan::primitive::scalar::Integer16_1_0 uavcan_motor0_current;
    uavcan_motor0_current.value = ads1115_data0;
    if(motor0_current_pub) motor0_current_pub->publish(uavcan_motor0_current);

    prev_motor0_current = now;
  }
  if((now - prev_motor1_current) > update_period_ms_motor1_current)
  {
    uavcan::primitive::scalar::Integer16_1_0 uavcan_motor1_current;
    uavcan_motor1_current.value = ads1115_data1;
    if(motor1_current_pub) motor1_current_pub->publish(uavcan_motor1_current);

    prev_motor1_current = now;
  }
  if((now - prev_motor0_bemf) > update_period_ms_motor0_bemf)
  {
    uavcan::primitive::scalar::Integer16_1_0 uavcan_motor0_bemf;
    uavcan_motor0_bemf.value = ads1115_data2;
    if(motor0_bemf_pub) motor0_bemf_pub->publish(uavcan_motor0_bemf);

    prev_motor0_bemf = now;
  }
  if((now - prev_motor1_bemf) > update_period_ms_motor1_bemf)
  {
    uavcan::primitive::scalar::Integer16_1_0 uavcan_motor1_bemf;
    uavcan_motor1_bemf.value = ads1115_data3;
    if(motor1_bemf_pub) motor1_bemf_pub->publish(uavcan_motor1_bemf);

    prev_motor1_bemf = now;
  }
  if((now - prev_encoder0) > update_period_ms_encoder0)
  {
    uavcan::primitive::scalar::Integer32_1_0 uavcan_encoder0;
    uavcan_encoder0.value = encoder0.getCount();
    if(encoder0_pub) encoder0_pub->publish(uavcan_encoder0);

    prev_encoder0 = now;
  }
  if((now - prev_encoder1) > update_period_ms_encoder1)
  {
    uavcan::primitive::scalar::Integer32_1_0 uavcan_encoder1;
    uavcan_encoder1.value = encoder1.getCount();
    if(encoder1_pub) encoder1_pub->publish(uavcan_encoder1);

    prev_encoder1 = now;
  }
  /* Feed the watchdog only if not an async reset is
   * pending because we want to restart via yakut.
   */
  if (!cyphal::support::platform::is_async_reset_pending())
    rp2040.wdt_reset();
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  node_hdl.onCanFrameReceived(frame);
}

ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const & req)
{
  ExecuteCommand::Response_1_1 rsp;

  if (req.command == ExecuteCommand::Request_1_1::COMMAND_RESTART)
  {
    if (auto const opt_err = cyphal::support::platform::reset_async(std::chrono::milliseconds(1000)); opt_err.has_value())
    {
      DBG_ERROR("reset_async failed with error code %d", static_cast<int>(opt_err.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_STORE_PERSISTENT_STATES)
  {
    if (auto const err_mount = filesystem.mount(); err_mount.has_value())
    {
      DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    /* Feed the watchdog. */
    rp2040.wdt_reset();
#if __GNUC__ >= 11
    auto const rc_save = cyphal::support::save(kv_storage, *node_registry, []() { rp2040.wdt_reset(); });
    if (rc_save.has_value())
    {
      DBG_ERROR("cyphal::support::save failed with %d", static_cast<int>(rc_save.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    /* Feed the watchdog. */
    rp2040.wdt_reset();
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
#endif /* __GNUC__ >= 11 */
    (void)filesystem.unmount();
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_POWER_OFF)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_BEGIN_SOFTWARE_UPDATE)
  {
    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
    /* not implemented yet */
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_FACTORY_RESET)
  {
    /* erasing eeprom by writing FF in every cell */
    size_t const NUM_PAGES = eeprom.device_size() / eeprom.page_size();
    for(size_t page = 0; page < NUM_PAGES; page++)
    {
      uint16_t const page_addr = page * eeprom.page_size();
      eeprom.fill_page(page_addr, 0xFF);
      rp2040.wdt_reset();
    }

    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_EMERGENCY_STOP)
  {
    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
    /* not implemented yet */
  }
  else {
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
  }

  return rsp;
}
