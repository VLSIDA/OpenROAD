%module mesh

%{
%}

%include "../../Exception.i"

%inline %{
namespace mesh {

void run_cmd(const char* /*name*/) {}

bool has_clock_mesh()
{
  return false;
}

}  // namespace mesh
%}
