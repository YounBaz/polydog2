project('coinhsl', 'fortran', 'c',
  version : '2023.05.26',
  default_options : ['warning_level=3'])

src = []
headers = []
include_dirs = []
deps = []

fs = import('fs')
full_install = fs.is_dir('hsl_ma97')

dirs = ['common', 'ma27', 'ma28', 'mc19']
if full_install
  dirs += ['hsl_ma77', 'hsl_ma86', 'hsl_ma97', 'hsl_mc68', 'ma57']
endif
foreach name : dirs
  subdir(name)
endforeach

if full_install
  libblas_name = get_option('libblas')
  libblas_path = get_option('libblas_path')

  liblapack_name = get_option('liblapack')
  liblapack_path = get_option('liblapack_path')

  libmetis_name = get_option('libmetis')
  libmetis_path = get_option('libmetis_path')
  libmetis_include = get_option('libmetis_include')
  libmetis_version = get_option('libmetis_version')

  libomp = dependency('openmp', language : 'fortran', required: true)

  fc = meson.get_compiler('fortran')
  libblas = fc.find_library(libblas_name, dirs : libblas_path, required : true)
  liblapack = fc.find_library(liblapack_name, dirs : liblapack_path, required : true)

  libmetis = fc.find_library(libmetis_name, dirs : libmetis_path, required : false)

  if fc.get_id() == 'gcc' and fc.version().version_compare('>=10.0')
    add_global_arguments('-fallow-argument-mismatch', language : 'fortran')
  endif

  libhsl_src = []
  subdir('metis')
  src +=  libhsl_src

  include_dirs += get_option('libmetis_include')


  deps += [libblas, liblapack, libomp, libmetis]
endif

library('coinhsl',
        sources : src,
	      dependencies : deps,
        include_directories : include_dirs,
        install : true)

install_headers(headers)
