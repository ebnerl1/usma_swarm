#!/bin/sh
#Generate xml from input file in a predefined format
#

NOFILE=64
[ -z $1 ] &&  echo "xmlgen.sh <filename>" && exit $NOFILE

myfile=${1}
basefile=$(basename $myfile .txt)

#Construct the frame
header='<?xml version="1.0"?>
<?xml-stylesheet type="text/xsl" href="http://gtri.gatech.edu"?>
<runscript xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"           
    name="USMA Scenario Test">
  
  <run start="0.0" end="1000" dt="0.1" 
       time_warp="1" 
       enable_gui="true" 
       start_paused="true"/>
  
  <end_condition>time, one_team</end_condition> <!-- time, one_team, none-->  
  
  <terrain>mcmillan</terrain>  
  <background_color>191 191 191</background_color> <!-- Red Green Blue -->
  <gui_update_period>10</gui_update_period> <!-- milliseconds -->
  
  <plot_tracks>false</plot_tracks>
  <output_type weights_file="sasc">all</output_type>
  <show_plugins>false</show_plugins>
  
  <log_dir>~/swarm-log</log_dir>  
      
  <latitude_origin>35.721025</latitude_origin>
  <longitude_origin>-120.767925</longitude_origin>
  <altitude_origin>300</altitude_origin>
  <show_origin>false</show_origin>
  
  <entity_interaction order="0">CaptureTheFlagInteraction</entity_interaction>
  
  <!-- uncomment "seed" and use integer for deterministic results -->
  <!--<seed>0</seed>-->
'

fmt='  <!-- ========================== TEAM 1 ========================= -->
  <entity>
    <team_id>%d</team_id> 
    <color>77 77 255</color>
    <count>%d</count>
    <health>1</health>

    <generate_rate> 1 / 2 </generate_rate>
    <generate_count>2</generate_count>
    <generate_start_time>0</generate_start_time>
    <generate_time_variance>0.10</generate_time_variance>

    <variance_x>50</variance_x>
    <variance_y>50</variance_y>
    <variance_z>50</variance_z>
    <latitude>35.722483</latitude>
    <longitude>-120.770014</longitude>
    <altitude>500</altitude>
    <heading>0</heading>      

    <controller>SimpleAircraftControllerPID</controller>
    <motion_model>SimpleAircraft</motion_model>
    <visual_model>wing.obj</visual_model>
    <texture>wing-blue.png</texture>
    <visual_scale>0.004598789</visual_scale>
    <visual_rotate>90 0 0 1</visual_rotate>
    <autonomy>%s</autonomy>

    <base>
      <latitude>35.722483</latitude>
      <longitude>-120.770014</longitude>
      <altitude>300</altitude>
      <radius>25</radius>
    </base>
  </entity>

  <entity>
    <team_id>%d</team_id>
    <color>77 77 255</color>
    <count>%d</count>
    <health>1</health>

    <generate_rate> 1 / 2 </generate_rate>
    <generate_count>2</generate_count>
    <generate_start_time>0</generate_start_time>
    <generate_time_variance>0.10</generate_time_variance>

    <variance_x>50</variance_x>
    <variance_y>50</variance_y>
    <variance_z>50</variance_z>
    <latitude>35.722483</latitude>
    <longitude>-120.770014</longitude>
    <altitude>500</altitude>
    <heading>0</heading>

    <controller>SimpleQuadrotorControllerLQR</controller>
    <motion_model>SimpleQuadrotor</motion_model>
    <visual_model>iris.obj</visual_model>
    <visual_scale>10</visual_scale>
    <visual_rotate>180 0 0 1</visual_rotate>
    <autonomy>%s</autonomy>
  </entity>

  <!-- ========================== TEAM 2 ========================= -->
  <entity>
    <team_id>%d</team_id>
    <color>255 0 0</color>
    <count>%d</count>
    <health>1</health>

    <generate_rate> 1 / 2 </generate_rate>
    <generate_count>2</generate_count>
    <generate_start_time>0</generate_start_time>
    <generate_time_variance>0.10</generate_time_variance>

    <variance_x>50</variance_x>
    <variance_y>50</variance_y>
    <variance_z>50</variance_z>
    <latitude>35.721072</latitude>
    <longitude>-120.766184</longitude>
    <altitude>500</altitude>
    <heading>180</heading>

    <controller>SimpleAircraftControllerPID</controller>
    <motion_model>SimpleAircraft</motion_model>
    <visual_model>wing.obj</visual_model>
    <texture>wing-red.png</texture>
    <visual_scale>0.004598789</visual_scale>
    <visual_rotate>90 0 0 1</visual_rotate>
    <autonomy>%s</autonomy>
  
    <base>
      <latitude>35.721072</latitude>
      <longitude>-120.766184</longitude>
      <altitude>300</altitude>
      <radius>25</radius>
    </base>
  </entity>

  <entity>
    <team_id>%d</team_id> 
    <color>255 0 0</color>
    <count>%d</count>
    <health>1</health>

    <generate_rate> 1 / 2 </generate_rate>
    <generate_count>2</generate_count>
    <generate_start_time>0</generate_start_time>
    <generate_time_variance>0.10</generate_time_variance>

    <variance_x>50</variance_x>
    <variance_y>50</variance_y>
    <variance_z>50</variance_z>
    <latitude>35.721072</latitude>
    <longitude>-120.766184</longitude>
    <altitude>500</altitude>
    <heading>180</heading>

    <controller>SimpleQuadrotorControllerLQR</controller>
    <motion_model>SimpleQuadrotor</motion_model>
    <visual_model>iris.obj</visual_model>
    <visual_scale>10</visual_scale>
    <visual_rotate>180 0 0 1</visual_rotate>
    <autonomy>%s</autonomy>
  </entity>

'

footer='</runscript>'

#Printing
{
printf "%s\n" "$header"
  while read team1 num1 behavior1 team2 num2 behavior2 team3 num3 behavior3 team4 num4 behavior4
     do
        printf "$fmt" "$team1" "$num1" "$behavior1" "$team2" "$num2" "$behavior2" "$team3" "$num3" "$behavior3" "$team4" "$num4" "$behavior4"
     done < "$myfile"
printf "%s\n" "$footer"
} > $basefile.xml
