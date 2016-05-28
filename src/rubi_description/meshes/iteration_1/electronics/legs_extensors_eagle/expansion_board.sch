<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="7.5.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="10" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="26" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="con-molex">
<description>&lt;b&gt;Molex Connectors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="52746-11">
<description>&lt;b&gt;FPC Connector ZIF for SMT 0.5mm&lt;/b&gt;&lt;p&gt;
Source: http://www.farnell.com/datasheets/73044.pdf [DWG. NO. (Sheet 1 OF 2) SD-52746-**17]</description>
<wire x1="-4.8" y1="0.6" x2="-3" y2="0.6" width="0.2032" layer="21"/>
<wire x1="-3" y1="0.6" x2="3" y2="0.6" width="0.2032" layer="51"/>
<wire x1="3" y1="0.6" x2="4.8" y2="0.6" width="0.2032" layer="21"/>
<wire x1="4.8" y1="0.6" x2="4.8" y2="-4" width="0.2032" layer="51"/>
<wire x1="4.8" y1="-4" x2="4.1" y2="-4" width="0.2032" layer="21"/>
<wire x1="4.1" y1="-4" x2="4.1" y2="-5" width="0.2032" layer="21"/>
<wire x1="4.1" y1="-5" x2="5.7" y2="-5" width="0.2032" layer="21"/>
<wire x1="5.7" y1="-5" x2="5.7" y2="-5.2" width="0.2032" layer="21"/>
<wire x1="5.7" y1="-5.2" x2="5.3" y2="-5.6" width="0.2032" layer="21" curve="-90"/>
<wire x1="5.3" y1="-5.6" x2="-5.3" y2="-5.6" width="0.2032" layer="21"/>
<wire x1="-5.3" y1="-5.6" x2="-5.7" y2="-5.2" width="0.2032" layer="21" curve="-90"/>
<wire x1="-5.7" y1="-5.2" x2="-5.7" y2="-5" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="-5" x2="-4.1" y2="-5" width="0.2032" layer="21"/>
<wire x1="-4.1" y1="-5" x2="-4.1" y2="-4" width="0.2032" layer="21"/>
<wire x1="-4.1" y1="-4" x2="-4.8" y2="-4" width="0.2032" layer="21"/>
<wire x1="-4.8" y1="-4" x2="-4.8" y2="0.6" width="0.2032" layer="51"/>
<wire x1="-5.75" y1="-0.25" x2="-4.95" y2="-0.25" width="0.5" layer="51"/>
<wire x1="-4.95" y1="-0.25" x2="-4.95" y2="-1.4" width="0.5" layer="51"/>
<wire x1="-4.95" y1="-1.4" x2="-5.75" y2="-1.4" width="0.5" layer="51"/>
<wire x1="5.75" y1="-1.4" x2="4.95" y2="-1.4" width="0.5" layer="51"/>
<wire x1="4.95" y1="-1.4" x2="4.95" y2="-0.25" width="0.5" layer="51"/>
<wire x1="4.95" y1="-0.25" x2="5.75" y2="-0.25" width="0.5" layer="51"/>
<smd name="1" x="-2.5" y="0.6" dx="0.3" dy="0.8" layer="1" stop="no"/>
<smd name="2" x="-2" y="0.6" dx="0.3" dy="0.8" layer="1" stop="no"/>
<smd name="3" x="-1.5" y="0.6" dx="0.3" dy="0.8" layer="1" stop="no"/>
<smd name="4" x="-1" y="0.6" dx="0.3" dy="0.8" layer="1" stop="no"/>
<smd name="5" x="-0.5" y="0.6" dx="0.3" dy="0.8" layer="1" stop="no"/>
<smd name="6" x="0" y="0.6" dx="0.3" dy="0.8" layer="1" stop="no"/>
<smd name="7" x="0.5" y="0.6" dx="0.3" dy="0.8" layer="1" stop="no"/>
<smd name="8" x="1" y="0.6" dx="0.3" dy="0.8" layer="1" stop="no"/>
<smd name="9" x="1.5" y="0.6" dx="0.3" dy="0.8" layer="1" stop="no"/>
<smd name="10" x="2" y="0.6" dx="0.3" dy="0.8" layer="1" stop="no"/>
<smd name="11" x="2.5" y="0.6" dx="0.3" dy="0.8" layer="1" stop="no"/>
<text x="-3.465" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-4.735" y="-7.62" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-6.15" y1="-2" x2="-3.75" y2="0.2" layer="1"/>
<rectangle x1="-5.05" y1="-2.85" x2="-3.75" y2="-2" layer="1"/>
<rectangle x1="3.75" y1="-2" x2="6.15" y2="0.2" layer="1"/>
<rectangle x1="3.75" y1="-2.85" x2="5.05" y2="-2" layer="1"/>
<rectangle x1="-6.15" y1="-2" x2="-3.75" y2="0.2" layer="29"/>
<rectangle x1="-5.05" y1="-2.85" x2="-3.75" y2="-2" layer="29"/>
<rectangle x1="3.75" y1="-2" x2="6.15" y2="0.2" layer="29"/>
<rectangle x1="3.75" y1="-2.85" x2="5.05" y2="-2" layer="29"/>
<rectangle x1="-2.675" y1="0.175" x2="-2.325" y2="1.025" layer="29"/>
<rectangle x1="-2.175" y1="0.175" x2="-1.825" y2="1.025" layer="29"/>
<rectangle x1="-1.675" y1="0.175" x2="-1.325" y2="1.025" layer="29"/>
<rectangle x1="-1.175" y1="0.175" x2="-0.825" y2="1.025" layer="29"/>
<rectangle x1="-0.675" y1="0.175" x2="-0.325" y2="1.025" layer="29"/>
<rectangle x1="-0.175" y1="0.175" x2="0.175" y2="1.025" layer="29"/>
<rectangle x1="0.325" y1="0.175" x2="0.675" y2="1.025" layer="29"/>
<rectangle x1="0.825" y1="0.175" x2="1.175" y2="1.025" layer="29"/>
<rectangle x1="1.325" y1="0.175" x2="1.675" y2="1.025" layer="29"/>
<rectangle x1="1.825" y1="0.175" x2="2.175" y2="1.025" layer="29"/>
<rectangle x1="2.325" y1="0.175" x2="2.675" y2="1.025" layer="29"/>
<rectangle x1="-6.025" y1="-1.9" x2="-3.85" y2="0.075" layer="31"/>
<rectangle x1="-4.95" y1="-2.75" x2="-3.85" y2="-1.9" layer="31"/>
<rectangle x1="3.85" y1="-1.9" x2="6.025" y2="0.075" layer="31"/>
<rectangle x1="3.85" y1="-2.75" x2="4.95" y2="-1.9" layer="31"/>
</package>
<package name="22-23-2081">
<description>.100" (2.54mm) Center Header - 8 Pin</description>
<wire x1="-10.16" y1="3.175" x2="10.16" y2="3.175" width="0.254" layer="21"/>
<wire x1="10.16" y1="3.175" x2="10.16" y2="1.27" width="0.254" layer="21"/>
<wire x1="10.16" y1="1.27" x2="10.16" y2="-3.175" width="0.254" layer="21"/>
<wire x1="10.16" y1="-3.175" x2="-10.16" y2="-3.175" width="0.254" layer="21"/>
<wire x1="-10.16" y1="-3.175" x2="-10.16" y2="1.27" width="0.254" layer="21"/>
<wire x1="-10.16" y1="1.27" x2="-10.16" y2="3.175" width="0.254" layer="21"/>
<wire x1="-10.16" y1="1.27" x2="10.16" y2="1.27" width="0.254" layer="21"/>
<pad name="1" x="-8.89" y="0" drill="1" shape="long" rot="R90"/>
<pad name="2" x="-6.35" y="0" drill="1" shape="long" rot="R90"/>
<pad name="3" x="-3.81" y="0" drill="1" shape="long" rot="R90"/>
<pad name="4" x="-1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="5" x="1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="6" x="3.81" y="0" drill="1" shape="long" rot="R90"/>
<pad name="7" x="6.35" y="0" drill="1" shape="long" rot="R90"/>
<pad name="8" x="8.89" y="0" drill="1" shape="long" rot="R90"/>
<text x="-10.16" y="3.81" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-10.16" y="-5.08" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="MV">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<text x="-0.762" y="1.397" size="1.778" layer="96">&gt;VALUE</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
<symbol name="M">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="52746-11" prefix="X">
<description>&lt;b&gt;FPC Connector ZIF for SMT 0.5mm&lt;/b&gt;&lt;p&gt;
Source: http://www.farnell.com/datasheets/73044.pdf [DWG. NO. (Sheet 1 OF 2) SD-52746-**17]</description>
<gates>
<gate name="-1" symbol="MV" x="-2.54" y="12.7" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="-2.54" y="10.16" addlevel="always" swaplevel="1"/>
<gate name="-3" symbol="M" x="-2.54" y="7.62" addlevel="always" swaplevel="1"/>
<gate name="-4" symbol="M" x="-2.54" y="5.08" addlevel="always" swaplevel="1"/>
<gate name="-5" symbol="M" x="-2.54" y="2.54" addlevel="always" swaplevel="1"/>
<gate name="-6" symbol="M" x="-2.54" y="0" addlevel="always" swaplevel="1"/>
<gate name="-7" symbol="M" x="-2.54" y="-2.54" addlevel="always" swaplevel="1"/>
<gate name="-8" symbol="M" x="-2.54" y="-5.08" addlevel="always" swaplevel="1"/>
<gate name="-9" symbol="M" x="-2.54" y="-7.62" addlevel="always" swaplevel="1"/>
<gate name="-10" symbol="M" x="-2.54" y="-10.16" addlevel="always" swaplevel="1"/>
<gate name="-11" symbol="M" x="-2.54" y="-12.7" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="" package="52746-11">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-10" pin="S" pad="10"/>
<connect gate="-11" pin="S" pad="11"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
<connect gate="-8" pin="S" pad="8"/>
<connect gate="-9" pin="S" pad="9"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="22-23-2081" prefix="X">
<description>.100" (2.54mm) Center Header - 8 Pin</description>
<gates>
<gate name="-1" symbol="MV" x="0" y="7.62" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="0" y="5.08" addlevel="always" swaplevel="1"/>
<gate name="-3" symbol="M" x="0" y="2.54" addlevel="always" swaplevel="1"/>
<gate name="-4" symbol="M" x="0" y="0" addlevel="always" swaplevel="1"/>
<gate name="-5" symbol="M" x="0" y="-2.54" addlevel="always" swaplevel="1"/>
<gate name="-6" symbol="M" x="0" y="-5.08" addlevel="always" swaplevel="1"/>
<gate name="-7" symbol="M" x="0" y="-7.62" addlevel="always" swaplevel="1"/>
<gate name="-8" symbol="M" x="0" y="-10.16" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="" package="22-23-2081">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
<connect gate="-8" pin="S" pad="8"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="22-23-2081" constant="no"/>
<attribute name="OC_FARNELL" value="1756826" constant="no"/>
<attribute name="OC_NEWARK" value="01C7592" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="INPUT" library="con-molex" deviceset="52746-11" device=""/>
<part name="OUTPUT" library="con-molex" deviceset="22-23-2081" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="INPUT" gate="-1" x="45.72" y="38.1" rot="R180"/>
<instance part="INPUT" gate="-2" x="45.72" y="40.64" rot="R180"/>
<instance part="INPUT" gate="-3" x="45.72" y="43.18" rot="R180"/>
<instance part="INPUT" gate="-4" x="45.72" y="45.72" rot="R180"/>
<instance part="INPUT" gate="-5" x="45.72" y="48.26" rot="R180"/>
<instance part="INPUT" gate="-6" x="45.72" y="50.8" rot="R180"/>
<instance part="INPUT" gate="-7" x="45.72" y="53.34" rot="R180"/>
<instance part="INPUT" gate="-8" x="45.72" y="55.88" rot="R180"/>
<instance part="INPUT" gate="-9" x="45.72" y="58.42" rot="R180"/>
<instance part="INPUT" gate="-10" x="45.72" y="60.96" rot="R180"/>
<instance part="INPUT" gate="-11" x="45.72" y="63.5" rot="R180"/>
<instance part="OUTPUT" gate="-1" x="86.36" y="63.5"/>
<instance part="OUTPUT" gate="-2" x="86.36" y="60.96"/>
<instance part="OUTPUT" gate="-3" x="86.36" y="58.42"/>
<instance part="OUTPUT" gate="-4" x="86.36" y="55.88"/>
<instance part="OUTPUT" gate="-5" x="86.36" y="53.34"/>
<instance part="OUTPUT" gate="-6" x="86.36" y="50.8"/>
<instance part="OUTPUT" gate="-7" x="86.36" y="48.26"/>
<instance part="OUTPUT" gate="-8" x="86.36" y="45.72"/>
</instances>
<busses>
</busses>
<nets>
<net name="N$1" class="0">
<segment>
<pinref part="INPUT" gate="-1" pin="S"/>
<pinref part="OUTPUT" gate="-8" pin="S"/>
<wire x1="48.26" y1="38.1" x2="83.82" y2="38.1" width="0.1524" layer="91"/>
<wire x1="83.82" y1="38.1" x2="83.82" y2="45.72" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="INPUT" gate="-2" pin="S"/>
<wire x1="48.26" y1="40.64" x2="81.28" y2="40.64" width="0.1524" layer="91"/>
<wire x1="81.28" y1="40.64" x2="81.28" y2="48.26" width="0.1524" layer="91"/>
<pinref part="OUTPUT" gate="-7" pin="S"/>
<wire x1="81.28" y1="48.26" x2="83.82" y2="48.26" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="INPUT" gate="-3" pin="S"/>
<wire x1="48.26" y1="43.18" x2="78.74" y2="43.18" width="0.1524" layer="91"/>
<wire x1="78.74" y1="43.18" x2="78.74" y2="50.8" width="0.1524" layer="91"/>
<pinref part="OUTPUT" gate="-6" pin="S"/>
<wire x1="78.74" y1="50.8" x2="83.82" y2="50.8" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="INPUT" gate="-4" pin="S"/>
<wire x1="48.26" y1="45.72" x2="76.2" y2="45.72" width="0.1524" layer="91"/>
<wire x1="76.2" y1="45.72" x2="76.2" y2="53.34" width="0.1524" layer="91"/>
<pinref part="OUTPUT" gate="-5" pin="S"/>
<wire x1="76.2" y1="53.34" x2="83.82" y2="53.34" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="INPUT" gate="-5" pin="S"/>
<wire x1="48.26" y1="48.26" x2="73.66" y2="48.26" width="0.1524" layer="91"/>
<wire x1="73.66" y1="48.26" x2="73.66" y2="55.88" width="0.1524" layer="91"/>
<pinref part="OUTPUT" gate="-4" pin="S"/>
<wire x1="73.66" y1="55.88" x2="83.82" y2="55.88" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="INPUT" gate="-6" pin="S"/>
<pinref part="INPUT" gate="-7" pin="S"/>
<wire x1="48.26" y1="50.8" x2="48.26" y2="53.34" width="0.1524" layer="91"/>
<wire x1="48.26" y1="53.34" x2="71.12" y2="53.34" width="0.1524" layer="91"/>
<wire x1="71.12" y1="53.34" x2="71.12" y2="58.42" width="0.1524" layer="91"/>
<pinref part="OUTPUT" gate="-3" pin="S"/>
<wire x1="71.12" y1="58.42" x2="83.82" y2="58.42" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="INPUT" gate="-8" pin="S"/>
<pinref part="INPUT" gate="-9" pin="S"/>
<wire x1="48.26" y1="55.88" x2="48.26" y2="58.42" width="0.1524" layer="91"/>
<wire x1="48.26" y1="58.42" x2="68.58" y2="58.42" width="0.1524" layer="91"/>
<wire x1="68.58" y1="58.42" x2="68.58" y2="60.96" width="0.1524" layer="91"/>
<pinref part="OUTPUT" gate="-2" pin="S"/>
<wire x1="68.58" y1="60.96" x2="83.82" y2="60.96" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="INPUT" gate="-10" pin="S"/>
<pinref part="INPUT" gate="-11" pin="S"/>
<wire x1="48.26" y1="60.96" x2="48.26" y2="63.5" width="0.1524" layer="91"/>
<pinref part="OUTPUT" gate="-1" pin="S"/>
<wire x1="48.26" y1="63.5" x2="83.82" y2="63.5" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
