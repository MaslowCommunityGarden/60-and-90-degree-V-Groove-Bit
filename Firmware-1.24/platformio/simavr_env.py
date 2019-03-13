from SCons.Script import ARGUMENTS, AlwaysBuild

Import('env')

# Run the linker with "-g", to prevent stripping of debugging symbols
env.Append(
  LINKFLAGS=[
      "-g"
  ]
)

# Don't try to upload the firmware
env.Replace(UPLOADHEXCMD="echo Upload is not supported for ${PIOENV}. Skipping")

pioenv = env.get("PIOENV")
progname = env.get("PROGNAME")

def simulate_callback(*args, **kwargs):
    env.Execute("./simduino/simduino .pioenvs/" + pioenv + "/" + progname + ".elf")

AlwaysBuild(env.Alias("simulate", "", simulate_callback))
