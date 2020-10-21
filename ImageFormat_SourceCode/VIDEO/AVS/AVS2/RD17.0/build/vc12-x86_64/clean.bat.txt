@echo off
SET dict=%~dp0

for /F %%i in ('dir %dict% /b ^|findstr /i /v ".bat$"') do ( 
    rd /s /q "%dict%\%%i" 2>nul || del /f /q "%dict%\%%i" 2>nul 
)

del "%dict%..\..\source\bin\*.exe" 2>nul 1>nul
del "%dict%..\..\source\bin\*.pdb" 2>nul 1>nul
del "%dict%..\..\source\bin\*.ilk" 2>nul 1>nul
