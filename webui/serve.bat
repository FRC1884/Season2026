@echo off
setlocal
set PORT=%1
if "%PORT%"=="" set PORT=8000
set BIND=%2
if "%BIND%"=="" set BIND=0.0.0.0

pushd %~dp0
echo Serving webui from: %CD%

where python >NUL 2>NUL
if %ERRORLEVEL%==0 (
  echo Launching: python -m http.server %PORT% --bind %BIND%
  python -m http.server %PORT% --bind %BIND%
  goto :eof
)

where py >NUL 2>NUL
if %ERRORLEVEL%==0 (
  echo Launching: py -3 -m http.server %PORT% --bind %BIND%
  py -3 -m http.server %PORT% --bind %BIND%
  goto :eof
)

where npx >NUL 2>NUL
if %ERRORLEVEL%==0 (
  echo Launching: npx http-server -a %BIND% -p %PORT%
  npx -y http-server -a %BIND% -p %PORT%
  goto :eof
)

echo No static server found (python/py/node/npx). Install Python or Node.js, or host this folder with IIS.
pause
popd

