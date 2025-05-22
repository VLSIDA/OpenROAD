# Copyright (c) 2021, The Regents of the University of California
# All rights reserved.
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

sta::define_cmd_args "set_cms_value" {[-value value]}

# Put helper functions in a separate namespace so they are not visible
# too users in the global namespace.

proc set_cms_value { args } {
  sta::parse_key_args "set_cms_value" args \
  keys {-value }

  if { [info exists keys(-value)]} {
    set value $keys(-value)
    cms::set_value $value
  }
  return [cms::dump_value]
}


