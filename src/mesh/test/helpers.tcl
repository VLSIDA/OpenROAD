# Helper functions for mesh unit tests

if { [info exists ::env(TEST_TMPDIR)] } {
  set test_dir $::env(TEST_TMPDIR)
} else {
  set test_dir [file dirname [file normalize [info script]]]
}

if { [info exists ::env(RESULTS_DIR)] } {
  set result_dir $::env(RESULTS_DIR)
} else {
  set result_dir [file join $test_dir "results"]
}

proc make_result_dir { } {
  variable result_dir
  if { ![file exists $result_dir] } {
    file mkdir $result_dir
  }
  return $result_dir
}

proc make_result_file { filename } {
  variable result_dir
  make_result_dir
  set root [file rootname $filename]
  set ext [file extension $filename]
  set filename "$root-tcl$ext"
  return [file join $result_dir $filename]
}

proc diff_files { file1 file2 } {
  set stream1 [open $file1 r]
  set stream2 [open $file2 r]

  set line 1
  set found_diff 0
  set line1_length [gets $stream1 line1]
  set line2_length [gets $stream2 line2]
  while { $line1_length >= 0 && $line2_length >= 0 } {
    if { $line1 != $line2 } {
      set found_diff 1
      break
    }
    incr line
    set line1_length [gets $stream1 line1]
    set line2_length [gets $stream2 line2]
  }
  close $stream1
  close $stream2
  if { $found_diff || $line1_length != $line2_length } {
    puts "Differences found at line $line."
    puts "$line1"
    puts "$line2"
    return 1
  } else {
    puts "No differences found."
    return 0
  }
}

set ::failing_checks 0
set ::passing_checks 0

proc check { description test expected_value } {
  if { [catch { set return_value [uplevel 1 $test] } msg] } {
    incr ::failing_checks
    error "FAIL: $description: Command \{$test\}\n$msg"
  } elseif { $return_value != $expected_value } {
    incr ::failing_checks
    error "FAIL: $description: Expected $expected_value, got $return_value"
  } else {
    incr ::passing_checks
  }
}

proc exit_summary { } {
  set total_checks [expr { $::passing_checks + $::failing_checks }]
  if { $total_checks > 0 } {
    set pass_per [expr { round(100.0 * $::passing_checks / $total_checks) }]
    puts "Summary $::passing_checks / $total_checks (${pass_per}% pass)"
    if { $total_checks == $::passing_checks } {
      puts "pass"
    }
  } else {
    puts "Summary 0 checks run"
  }
  exit $::failing_checks
}

# Suppress common messages
suppress_message ODB 127
suppress_message ODB 134
suppress_message ORD 30
