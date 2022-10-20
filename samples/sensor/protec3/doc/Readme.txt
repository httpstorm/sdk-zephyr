п»їMr. Angelov on Slack on 2021/01/28

I was working on thingy:91 board so a change on line 89 should be done
sw0 should become button
If you want to test it - you have to use some software mqtt client
I am using mqtt explorer and mqttbox
I'm sending the proper certificates too (bearbeitet)

3 Dateien
ca.crt
client.crt
client.key

host: protec3.io
port: 5883
certificate verification and tls: on
user: protec3.io
password: kR59Tq72d%.SX}4aze!^1KX_+nJBRy&T
If you want to test the firmware directly, follow this procedure - connect you software client and subscribe to all topics. When you power on the device - led0 will blink, nothing will happen until you press the button, then the device will initialize and make the mqtt connection. It will publish to a topic called /{device_id}/status, a message ONLINE
If you press the button the device will publish a message ON to a topic /{device_id}/button
If you publish a message ON to a topic /{device_id}/led
it will turn on led0
and message OFF will turn it off
I think that for now we should define the following topics:
device should publish gnss location data to /{device_id}/gnss/raw
it should publish to /{device_id}/alarm/status (bearbeitet)
messages should be ON and OFF according to the alarm status
publish to /{device_id}/button
message ON when the button is pressed
publish to /{device_id}/battery/voltage
with the battery voltage in mAh (bearbeitet)
publish to /{device_id}/battery/status
message should be the hex data from the status registers
device must subscribe to /{device_id}/alarm/arm (bearbeitet)
and to /{device_id}/alarm/status (bearbeitet)
on /status it should wait for GET and return ON or OFF accordingly
on /alarm/arm it should wait for GET, ON or OFF, and it should publish back the status or arm/disarm the alarm and publish back the status (bearbeitet)
for the bluetooth it should subscribe to /{device_id}/bluetooth/list, /{device_id}/add, /{device_id}/delete
on /list the device should expect only GET, and it should return a mac addresses list separated by commas
on /add and /delete it should expect a mac address
I think that the format of the mac address should be 16 characters long hex string
let's say upper case
this list will be for the phones that should not trigger the alarm if it is armed
device should publish on /{device_id}/status
ONLINE on connection, and OFFLINE when it disconnects
also lats will should publish OFFLINE on the same topic
on connection device should publish in all status topics - battery voltage, battery status, alarm status, bluetooth list and so on
bluetooth list should be stored in the eeprom, and I think that the alarm arm status should be stored too
also it is good to have uptime of the device, for which a message GET to the topic /{device_id}/uptime can be send whit a response of uptime in seconds
also it is good to have control over the leds on topics /{device_id}/led/0, /{device_id}/led/1, /{device_id}/led/2
again with messages ON, OFF, GET (bearbeitet)
and on the speaker let say on topic /{device_id}/beep/loud, /{device_id}/beep/normal
message should be the length of the sound in ms
its good to have temperature too, periodically on /{device_id}/temperature (bearbeitet)
and accelerometer position too, on /{device_id}/accelerometer
so far I think it is enough
I will make a google drive structure of the messages and share it with you, and we will modify it further during the development

on connect
publish to all status topics

publish
/{device_id}/status/link		MQTT status		ONLINE OFFLINE
/{device_id}/gnss/raw			RAW GPS data
/{device_id}/status/alarm		alarm event		ON OFF
/{device_id}/button				button event	ON OFF
/{device_id}/battery/voltage	battery voltage	3700 mv
/{device_id}/battery/status		battery status	hex status registers
/{device_id}/temperature		temperature		17
/{device_id}/accelerometer		accelerometer	x,y,z

subscribe
/{device_id}/alarm				alarm arm		ON OFF GET	replay /status/alarm ON OFF, store in EEPROM
/{device_id}/beep/loud			beep loud		time ms		beep for time in ms, loud
/{device_id}/beep/normal		beep normal		time ms		beep for time in ms, normal
/{device_id}/led/[0-2]			LED control		ON OFF GET	replay /status/led/[0-2] ON OFF
/{device_id}/uptime				uptime			GET			replay /status/uptime in seconds

subscribe BLE					В§1
/{device_id}/bluetooth/list		BLE whitelist	GET			replay with MAC1, MAC2..., store in EEPROM
/{device_id}/add				add device		MAC			hex string, 16 characters upper case В§2
/{device_id}/delete				delete device	MAC			hex string, 16 characters upper case В§2
->
subscribe user					В§1				publish to	/status/user/[add|del|list]
/{device_id}/user/add			user add		MAC			hex string, 16 characters upper case В§2
/{device_id}/user/del			user delete		MAC			hex string, 16 characters upper case В§2
/{device_id}/user/list			user list		GET			replay with MAC1, MAC2..., store in EEPROM

В§1	We do not have access to the MAC of connected devices.
	This data is only accessible to the BLE chip nRF52840.
	White listed devices should not trigger an alarm if armed.
В§2	"00:11:22:33:44:55" 17 characters, or "001122334455" 12 characters.
	The rest of the application is already designed to use lower case hex numbers.
