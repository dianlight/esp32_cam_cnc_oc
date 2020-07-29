Import("env")

# please keep $SOURCE variable, it will be replaced with a path to firmware

# In-line command with arguments
env.Replace(
    UPLOADCMD="curl --progress-bar -T $SOURCE $UPLOAD_PORT"
)
