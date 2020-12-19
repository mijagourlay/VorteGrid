rem This script works around the bug causing a locked program database (pdb)
rem after each debug session with Visual Studio 2003 on Windows 7.
rem 
rem You will need the Handle utility from Windows Sysinternals for this script.
rem (1) Get it here and extract it into the folder containing this script:
rem     site: http://technet.microsoft.com/en-us/sysinternals/bb896655.aspx
rem     file: http://download.sysinternals.com/Files/Handle.zip
rem
rem To automate execution, open the properties dialog of your project in VS2003:
rem (2) Navigate to:     Build Events -> Pre-Build Event -> Command Line
rem (3) Enter this line: <path>\freepdb $(ProjectName)
rem     Replace <path> with the path to the folder containing this script.
rem
rem Have fun!
rem
rem 30/05/2010 support multiple handle instances of multiple debug sessions
rem 29/05/2010 initial version
rem freepdb@inot.de
rem
rem September 11, 2011 Koltaiga modified to support handle v3.46
rem
rem MiJaGourlay: Note that handle.exe must run as administrator.
rem Although you can use ACT to cause that to happen automatically, doing so
rem seems to make handle.exe run in a nested shell so its output does not get
rem back to this script.  So that does not work.
rem Note also that disabling the annoying UAC popup does not help.
rem
rem You want Visual Studio to run as administrator so then the script,
rem and therefore handle.exe, run as administrator.  Right-click
rem C:\Program Files (x86)\Microsoft Visual Studio .NET 2003\Common7\IDE\devenv.exe,
rem select Properties, Compatibility tab, and check "Run this program as an adminstrator".
rem This step is optional, though, since you'll end up using the VS version selector.
rem
rem You could make all SLN files open with 2003 but bypassing the VS version
rem selector causes other (2005, 2008, 2010, etc.) SLN files to fail to open properly.
rem
rem The remaining issue is to get the version selector
rem (C:\Program Files (x86)\Common Files\microsoft shared\MSEnv\VSLauncher.exe)
rem to run as admin.  Right-click the exe, select Properties, Compatibility tab,
rem and check "Run this program as an administrator".
rem
rem With latest KB's, trying to make the VS version selector run as admin, will not work:
rem VS2003 files silently fail to open via the version selector.
rem
rem Using Windows Update to UNinstall any update that claims to improve compatibility:
rem UNinstall KB2492386, KB2762895, KB2791765, KB2820331 to fix that issue.

if "%1"=="" (
  echo Usage: freepdb filename
  echo This will free all handles of VS2003 on filename.pdb
  echo Requires the Handle utility from Windows Sysinternals
  pause -1
  goto :eof
)

@rem Uncomment the "echo on" line to see output from this script, in the VS Output window, to help debug this script.
@rem @echo on

cd

@rem First try to delete any old PDB files.
@rem This will fail when VS holds the file handle, but will succeed for any PDB files whose handles have been released (which will happen after VS terminates).
del VorteGrid\Debug\%1*.pdb
del VorteGrid\Profile\%1*.pdb
del VorteGrid\ProfileWithoutTbb\%1*.pdb
del VorteGrid\Release\%1*.pdb

@rem As a fallback, try to rename the current PDB file.
@rem This will succeed even if VS holds a file handle to the PDB file.
rename VorteGrid\Debug\%1.pdb %1_%RANDOM%%RANDOM%.pdb
rename VorteGrid\Profile\%1.pdb %1_%RANDOM%%RANDOM%.pdb
rename VorteGrid\ProfileWithoutTbb\%1.pdb %1_%RANDOM%%RANDOM%.pdb
rename VorteGrid\Release\%1.pdb %1_%RANDOM%%RANDOM%.pdb

set PATH=%PATH%;"C:\Program Files\SysinternalsSuite"

@rem cd /d "%~dp0"

for /f "tokens=2-3 skip=5 delims=:" %%a in ('handle -p devenv.exe "%1.pdb"') do (
  for /f "tokens=1,4" %%c in ("%%a %%b") do (
    handle -p %%c -c %%d -y >NUL
  )
)

exit /b 0
