#!/usr/bin/perl

# NEEDS
# apt-get install libdevice-serialport-perl
# apt-get install libdigest-crc-perl
#

use strict;
use warnings;
use Device::SerialPort qw( :PARAM :STAT 0.07 );
use Digest::CRC;
use Statistics::Descriptive;

my $debug = 0;
my $PortName = '/dev/ttyUSB0';
my $sSend = "";
my $res = "";
my $send_count=0;
my $item;
my $scheck_crc;
#my $sStatus1 = chr(0x01).chr(0x10).chr(0x01).chr(0xec);
my $sStatus1 = chr(0x01).chr(0x10);
my $sStatus2 = chr(0x01).chr(0x11);
my $sStatus3 = chr(0x01).chr(0x12);
my $sStatus4 = chr(0x01).chr(0x13);
my $sStatus5 = chr(0x01).chr(0x14);
my $sStatus6 = chr(0x01).chr(0x15);
my $sStatus7 = chr(0x01).chr(0x16);
my $sStatus8 = chr(0x01).chr(0x17);
my $sStatus9 = chr(0x01).chr(0x18);
my $sEnable  = chr(0x01).chr(0x20).chr(0x24).chr(0x38).chr(0x79).chr(0x0D); #Tracking enable
my $sDisable = chr(0x01).chr(0x20).chr(0x24).chr(0x38).chr(0x6E).chr(0x0D); #Tracking disable
my $sStop    = chr(0x01).chr(0x20).chr(0x24).chr(0x39).chr(0x73).chr(0x0D); #Stop Motors
my $sRefA    = chr(0x01).chr(0x20).chr(0x24).chr(0x39).chr(0x35).chr(0x0D); #Reference MotorA
my $sRefB    = chr(0x01).chr(0x20).chr(0x24).chr(0x39).chr(0x36).chr(0x0D); #Reference MotorB
my $sMoveEW  = chr(0x01).chr(0x20).chr(0x24).chr(0x2C); #Set E-W + Neigung + 0x0D
my $sMoveNS  = chr(0x01).chr(0x20).chr(0x24).chr(0x30); #Set N-S
#
my @SunTracker=(0..255);
my @vars1;
#
my $PortObj = new Device::SerialPort($PortName) || die "Can't open $PortName: $!\n";
my $ctx = Digest::CRC->new(width=>16, init=>0xFFFF, xorout=>0x0000, refout=>1, poly=>0x8005, refin=>1, cont=>1); # MODBUS CRC16

my $params = $#ARGV + 1;
# get script name
my $scriptname = $0;
my $counter = 1;
if ($debug == 1) {
    print "Total args passed to $scriptname : $params\n";
    # Use loop to print all args stored in an array called @ARGV
    foreach my $a(@ARGV) {
	print "Arg # $counter : $a\n";
	$counter++;
    }
}
# subroutine crc calc
sub f_crc_modbus {
    my $n = scalar(@_);
    my $result=0;
    my $lsb = 0;
    my $msb = 0;

    foreach $item (@_) {
	$ctx->add($item);
	$result = $ctx->digest;
	$lsb = $result & 0x00ff;			#Alternative für den Byteswap: unpack(">i", $var); #https://perldoc.perl.org/functions/pack
	$msb = ($result & 0xff00) >> 8;
	$result = $item.chr($lsb).chr($msb);
    }

    return $result;
}

# subroutine read serial
sub read_serial {
    my $chars=0;
    my $buffer="";
    my $timeout = 3;
    while ($timeout>0) {
       my ($count,$saw)=$PortObj->read(255); # will read _up to_ 255 chars
       if ($count > 0) {
               $chars+=$count;
               $buffer.=$saw;

               # Check here to see if what we want is in the $buffer
               # say "last" if we find it
       }
       else {
               $timeout--;
       }
    }
    # CRC check
    my $read_length = length($buffer)-2;
    my $sread_crc = substr($buffer, $read_length);
    $buffer = substr($buffer, 0, $read_length);
    #
    $ctx->add($buffer);
    my $result = $ctx->digest;
    my $lsb = $result & 0x00ff;			#Alternative für den Byteswap: unpack(">i", $var); #https://perldoc.perl.org/functions/pack
    my $msb = ($result & 0xff00) >> 8;
    $scheck_crc = chr($lsb).chr($msb);
    if ( $sread_crc ne $scheck_crc ) {
	print "CRC Error !\n";
	@vars1=unpack("H*","$sread_crc");
	print "CRC(read):  @vars1 vs. ";
	@vars1=unpack("H*","$scheck_crc");
	print "CRC(calc):  @vars1\n";
    }
    return $buffer;
}

# subroutine split received string
sub eval_split {

    my $input = $_[0];
    #print $input."\n"
    my @Split = split('\$', $input);
    foreach my $SplitPart(@Split)
    {
	#print "$SplitPart\n";
	my $first = unpack("H*", substr($SplitPart,0,1));
	my $second = substr($SplitPart,1);
	my $dez = hex($first);
	if ($debug == 1) {
	    print "ID: $first (".$dez."d)  Value: $second \n";
	}
	$SunTracker[$dez]=$second;
    }

}


$PortObj->databits(8);
$PortObj->baudrate(19200);
$PortObj->parity("even");
$PortObj->stopbits(1);
$PortObj->read_char_time(0);		# don't wait for each character
$PortObj->read_const_time(10);		# 50 msecond per unfulfilled "read" call

#-----------------------------------
if ($params == 1) {

    my $sfunction = $ARGV[0];
    if ($sfunction eq "Enable") {
	print "Enable Tracking\n";
	$sSend = f_crc_modbus($sEnable);
	$send_count = $PortObj->write($sSend);
	$res = read_serial();
	print $res."\n";
	#-----------------------------------
	} elsif ($sfunction eq "Disable") {
	print "Disable Tracking\n";
	$sSend = f_crc_modbus($sDisable);
	$send_count = $PortObj->write($sSend);
	$res = read_serial();
	if ( $res eq chr(0x01).chr(0x20) ) {
	    print "Ok\n";
	} else {
	    print "NOk ".$res."\n";
	}
	#-----------------------------------
	} elsif ($sfunction eq "Stop") {
	print "Stop Motors\n";
	$sSend = f_crc_modbus($sStop);
	$send_count = $PortObj->write($sSend);
	$res = read_serial();
	print $res."\n";
	#-----------------------------------
	} elsif ($sfunction eq "RefA") {
	print "Reference Motor A\n";
	$sSend = f_crc_modbus($sRefA);
	$send_count = $PortObj->write($sSend);
	$res = read_serial();
	print $res."\n";
	#-----------------------------------
	} elsif ($sfunction eq "RefB") {
	print "Reference Motor B\n";
	$sSend = f_crc_modbus($sRefB);
	$send_count = $PortObj->write($sSend);
	$res = read_serial();
	print $res."\n";
	#-----------------------------------
	} elsif ($sfunction eq "WIND") {
	print "Go to WIND position\n";
	$sSend = f_crc_modbus($sDisable);	#Disable Tracking
	$send_count = $PortObj->write($sSend);
	$res = read_serial();
	print $res."\n";
	$sMoveEW=$sMoveEW."0".chr(0x0D);	#E-W = 0
	$sSend = f_crc_modbus($sMoveEW);
	$send_count = $PortObj->write($sSend);
	$res = read_serial();
	print $res."\n";
	$sMoveNS=$sMoveNS."90".chr(0x0D);
	$sSend = f_crc_modbus($sMoveNS);	#N-S = 90
	$send_count = $PortObj->write($sSend);
	$res = read_serial();
	print $res."\n";
	#-----------------------------------
	} elsif ($sfunction eq "SNOW") {
	print "Go to SNOW position\n";
	$sSend = f_crc_modbus($sDisable);	#Disable Tracking
	$send_count = $PortObj->write($sSend);
	$res = read_serial();
	print $res."\n";
	$sMoveEW=$sMoveEW."0".chr(0x0D);	#E-W = 0
	$sSend = f_crc_modbus($sMoveEW);
	$send_count = $PortObj->write($sSend);
	$res = read_serial();
	print $res."\n";
	$sMoveNS=$sMoveNS."15".chr(0x0D);
	$sSend = f_crc_modbus($sMoveNS);	#N-S = 15
	$send_count = $PortObj->write($sSend);
	$res = read_serial();
	print $res."\n";
	#-----------------------------------
	} elsif ($sfunction eq "STATS") {
	    my $reads = 30;
	    my @messwerte;
	    my $stat = Statistics::Descriptive::Full->new();
	    while ($reads>0) {
		    $sSend = f_crc_modbus($sStatus6);
		    $send_count = $PortObj->write($sSend);
		    $res = read_serial();
		    eval_split($res);
		    push(@messwerte,$SunTracker[119]);
		    $reads--;
		    sleep 1;
		}
	    if ($debug == 1) {
		print @messwerte;
	    }
	    $stat->add_data( @messwerte ) ;
	    # Returns the number of data items.
	    print 'Wind Messwerte: ',  $stat->count(), " | ";
	    # Returns the mean of the data.
	    print 'Mittelwert: ',  $stat->mean(), " | ";
	    # Sorts the data and returns the median value of the data.
	    print 'Median: ',  $stat->median(), " | ";
	    # Returns the harmonic mean of the data. 
	    # print 'Harmonischer Mittelwert: ', $stat->harmonic_mean(), "\n";
	    # Returns the geometric mean of the data.
	    # print 'Geometrischer Mittelwert: ', $stat->geometric_mean(), "\n";
	    # Returns the sum of the data.
	    # print 'Summe: ', $stat->sum(), "\n";
	    # Returns the variance of the data. Division by n-1 is used.
	    # print 'Varianz: ', $stat->variance(), "\n";
	    # Returns the standard deviation of the data. Division by n-1 is used.
	    # print 'Standardabweichung: ', $stat->standard_deviation(), "\n";
	    # Returns the minimum value of the data set.
	    print 'Min: ' , $stat->min(), " | ";
	    # Returns the index of the minimum value of the data set.
	    # print 'Index Minimum: ', $stat->mindex(), "\n";
	    # Returns the maximum value of the data set.
	    print 'Max: ', $stat->max(), "\n";
	    # Returns the index of the maximum value of the data set.
	    # print 'Index Maximum: ', $stat->maxdex(), "\n";
	    # Returns the sample range (max - min) of the data set.
	    # print 'Stichprobenbereich: ',  $stat->sample_range(), "\n";
	} else {
	print "Usage $scriptname [Enable|Disable|Stop|RefA|RefB|WIND|SNOW|STATS] or $scriptname [MoveEW|MoveNS] [Degree]\n"
	}
}
elsif ($params == 2) {
    my $sfunction = $ARGV[0];
    my $value = $ARGV[1];
    if ($sfunction eq "MoveEW" && $value >= -50 && $value <= 50) {
	print "Move E-W to $value degree\n";
	$sMoveEW=$sMoveEW.$value.chr(0x0D);
	$sSend = f_crc_modbus($sMoveEW);
	#@vars1=unpack("H*","$sSend");
	#print "CRC:  @vars1\n";
	$send_count = $PortObj->write($sSend);
	$res = read_serial();
	print $res."\n";
	#-----------------------------------
	} elsif ($sfunction eq "MoveNS" && $value >= 12 && $value <= 90) {
	print "Move N-S to $value degree\n";
	$sMoveNS=$sMoveNS.$value.chr(0x0D);
	$sSend = f_crc_modbus($sMoveNS);
	#@vars1=unpack("H*","$sSend");
	#print "CRC:  @vars1\n";
	$send_count = $PortObj->write($sSend);
	$res = read_serial();
	print $res."\n";
	} else {
	print "Usage $scriptname [Enable|Disable|Stop|RefA|RefB|WIND|SNOW|STATS] or $scriptname [MoveEW|MoveNS] [Degree]\n"
	}

}
else {					#Statusmeldungen abfragen
    #-----------------------------------
    $sSend = f_crc_modbus($sStatus1);
    #@vars1=unpack("H*","$sSend");
    #print "CRC:  @vars1\n";
    $send_count = $PortObj->write($sSend);
    #print "Send $send_count bytes\n";
    $res = read_serial();
    #print $res."\n";
    eval_split($res);
    #$PortObj->purge_all;
    #-----------------------------------
    $sSend = f_crc_modbus($sStatus2);
    $send_count = $PortObj->write($sSend);
    $res = read_serial();
    eval_split($res);
    #-----------------------------------
    $sSend = f_crc_modbus($sStatus3);
    $send_count = $PortObj->write($sSend);
    $res = read_serial();
    eval_split($res);
    #-----------------------------------
    $sSend = f_crc_modbus($sStatus4);
    $send_count = $PortObj->write($sSend);
    $res = read_serial();
    eval_split($res);
    #-----------------------------------
    $sSend = f_crc_modbus($sStatus5);
    $send_count = $PortObj->write($sSend);
    $res = read_serial();
    eval_split($res);
    #-----------------------------------
    $sSend = f_crc_modbus($sStatus6);
    $send_count = $PortObj->write($sSend);
    $res = read_serial();
    eval_split($res);
    #-----------------------------------
    $sSend = f_crc_modbus($sStatus7);
    $send_count = $PortObj->write($sSend);
    $res = read_serial();
    eval_split($res);
    #-----------------------------------
    $sSend = f_crc_modbus($sStatus8);
    $send_count = $PortObj->write($sSend);
    $res = read_serial();
    eval_split($res);
    #-----------------------------------
    $sSend = f_crc_modbus($sStatus9);
    $send_count = $PortObj->write($sSend);
    $res = read_serial();
    eval_split($res);
    #-----------------------------------
    print "Zeit (UTC): ".$SunTracker[175].":".$SunTracker[174].":".$SunTracker[173]." ".$SunTracker[176].".".$SunTracker[177].".".$SunTracker[178]." Sonnenaufgang: ".$SunTracker[158]." -untergang: ".$SunTracker[159]." \n";
    print "Zeit (Sun): ".$SunTracker[40].":".$SunTracker[41].":".$SunTracker[42]." Höhenwinkel (elevation): ".$SunTracker[191]." Stundenwinkel (azimut): ".$SunTracker[190]." \n";
    print "Volt: ".$SunTracker[39]." E-W: ".$SunTracker[44]." N-S: ".$SunTracker[48]." Status: ".$SunTracker[56]." Service: ".$SunTracker[58]." \n";
    print "Motor-A Strom: ".$SunTracker[47]." Pos-Ist: ".$SunTracker[45]." Pos-Soll: ".$SunTracker[46]." Remain: ".$SunTracker[63]." \n";
    print "Motor-B Strom: ".$SunTracker[51]." Pos-Ist: ".$SunTracker[49]." Pos-Soll: ".$SunTracker[50]." Remain: ".$SunTracker[64]." \n";
    print "Windspeed: ".$SunTracker[119]."\n";
}

$PortObj->close || die "failed to close";
undef $PortObj;
