pushd ..

rmdir    /s /q _UpgradeReport_Files
rmdir    /s /q Ogle\Vols
rmdir    /s /q ipch

for /d /r %%d in ( Debug_Win32 Profile Profile_Win32 ProfileWithoutTbb ProfileWithoutTbb_Win32 Release_Win32 ) do rmdir /s /q "%%d"

@rem UNSUFFIXED_DIRS are project subdirectories where _Platform is not part of the Target OutDir.
set UNSUFFIXED_DIRS=Core QdRender VorteGrid VortonFluid
for /d /r %%d in ( %UNSUFFIXED_DIRS% ) do rmdir /s /q "%%d\Debug"
for /d /r %%d in ( %UNSUFFIXED_DIRS% ) do rmdir /s /q "%%d\Release"

@rem Handle Debug and Release separately since External contains Debug and Release folders we do not want to erase.
@rem for /d /r %%d in ( Debug Release ) do rmdir /s /q "%%d"

del   /f /s /q *.obj
del   /f /s /q *.idb
del   /f /s /q vc*.pdb
del   /f /s /q BuildLog.htm
del   /f /s /q UpgradeLog*.XML
del   /f /s /q default.log
del   /f /s /q stdout-*.log
del   /f /s /q perf-*.csv

 del   /f /s /q *.user
 del   /f /s /q *.suo
 del   /f /s /q *.ncb

@rem Remove all empty directories
robocopy . . /s /move

popd

pause
