# VLAgraph 메인 페이지

로봇 AI 워크플로우 플랫폼 VLAgraph의 랜딩 페이지와 회원가입·로그인 API입니다.

## 이 저장소에서 실행하기

이 랜딩 페이지는 기존 그래프 앱과 분리해 `http://localhost:5174`에서 실행합니다. 로그인 API는 8001 포트(컨테이너 안에서는 8000), PostgreSQL은 Docker 내부 네트워크만 사용하므로 기존 앱의 `5173`, `8000`, `9173` 포트와 데이터에 영향을 주지 않습니다. 로그인하면 시연 화면 카드에서 기존 그래프 앱으로 이동합니다.

1. Docker Desktop을 실행한 뒤 이 폴더에서 환경 파일을 만듭니다.

   ```powershell
   Copy-Item .env.example .env
   ```

2. `.env`에서 `POSTGRES_PASSWORD`를 URL에 안전한 문자(영문자, 숫자, `-`, `_`)로 된 강한 값으로, `SECRET_KEY`를 32자 이상의 임의 값으로 바꿉니다. 비밀번호 재설정 메일의 링크 주소를 설정하려면 `FRONTEND_URL=http://localhost:5174`로 지정합니다. 초기 관리자 계정이 필요하면 `ADMIN_EMAILS`와 `ADMIN_INITIAL_PASSWORD`를 설정합니다. 가입자는 관리자의 승인이 있어야 로그인할 수 있습니다. 관리자는 로그인 후 대시보드의 **회원 가입 승인** 패널에서 신청 목록을 확인하고 승인·거절할 수 있습니다.
3. 저장소 루트에서 랜딩 페이지, 인증 API, 데이터베이스를 시작합니다.

   ```powershell
   docker compose --project-directory landing -f landing/docker-compose.yml up --build
   ```

4. `http://localhost:5174`에서 계정을 만들거나 로그인한 뒤 **시연 화면 열기**를 클릭합니다. 그래프 앱은 기존과 같이 `http://localhost:9173`에서 별도로 실행 중이어야 합니다.

종료할 때는 같은 폴더 지정과 Compose 파일로 `docker compose --project-directory landing -f landing/docker-compose.yml down`을 실행합니다. 계정 데이터는 `auth_data` 볼륨에 보관됩니다. 비밀번호 재설정 이메일을 사용하려면 `.env`의 SMTP 설정도 입력해야 합니다.

## Isaac Sim 실행 버튼 사용

로그인 화면의 기존 시연 카드 아래에 추가된 **시뮬레이션 화면 열기** 버튼은 Windows 호스트에서 실행되는 로컬 런처에 요청합니다. Docker 컨테이너에서 직접 Windows Isaac Sim 실행 파일을 실행하지 않도록 분리한 구성입니다.

1. `landing/.env`에 Docker 백엔드와 로컬 런처가 공유할 임의의 긴 토큰을 설정합니다.

   ```dotenv
   OMNIVERSE_LAUNCH_TOKEN=replace-with-a-long-random-launch-token
   ```

2. `scripts/start-isaac-sim.ps1`에서 Isaac Sim 실행 파일과 선택적으로 열 Stage/USD 경로를 수정합니다.

   ```powershell
   $Executable = "C:\Path\To\isaac-sim.bat"
   $ScenePath = "C:\Path\To\scene.usd"
   $ExtraArguments = @()
   ```

3. 새 PowerShell 창에서 토큰과 함께 로컬 런처를 실행합니다.

   ```powershell
   $env:OMNIVERSE_LAUNCH_TOKEN = "landing/.env와 동일한 토큰"
   .\scripts\start-omniverse-launcher.ps1
   ```

4. 랜딩 서비스와 기존 그래프 앱을 실행한 뒤 로그인하고 **시뮬레이션 화면 열기**를 누릅니다. 승인된 로그인 사용자만 실행 API를 호출할 수 있습니다.

로컬 런처는 `127.0.0.1:9188`에서만 요청을 받습니다. 실행 경로는 브라우저가 임의로 입력할 수 없으며, `start-isaac-sim.ps1`에서 운영자가 관리합니다. 런처가 실행 중이지 않으면 화면에 연결 실패 메시지가 표시됩니다.

## 구성

- **프런트엔드:** React 18, Vite
- **백엔드:** FastAPI, Uvicorn
- **데이터베이스:** PostgreSQL
- **인증:** JWT를 HTTP-only 쿠키로 전달하며, 비밀번호는 Argon2 기반 해시로 저장합니다. 관리자에게는 가입 회원 조회·승인·거절 UI가 표시됩니다.

## 프로젝트 파일 안내

### 백엔드 (`backend/`)

| 파일 | 역할 |
| --- | --- |
| `main.py` | FastAPI 앱과 회원가입·로그인·로그아웃·현재 사용자 조회 API를 정의합니다. 요청 검증, 비밀번호 해시 검증, JWT 발급 및 인증 쿠키 처리도 담당합니다. |
| `config.py` | 환경 변수와 애플리케이션 설정을 정의하고 검증합니다. 데이터베이스 주소, JWT 비밀 키, 쿠키 설정 등을 관리합니다. |
| `db.py` | 요청에 사용할 PostgreSQL 연결을 생성하는 의존성 함수를 제공합니다. |
| `schema.sql` | 회원 계정을 저장하는 `users` 테이블을 생성합니다. 새 DB를 처음 설정할 때 실행합니다. |
| `requirements.txt` | 백엔드 실행에 필요한 Python 패키지와 버전 범위를 나열합니다. |
| `__init__.py` | `backend` 폴더를 Python 패키지로 인식시킵니다. |
| `../.env.example` | 인증 백엔드와 프런트엔드 환경 변수 예시입니다. 실제 접속 정보와 비밀 키는 이 프로젝트의 `.env`에 설정합니다. |

### 프런트엔드 (`src/`)

| 파일 | 역할 |
| --- | --- |
| `main.jsx` | React 앱의 화면과 동작을 정의합니다. 랜딩 페이지, 회원가입·로그인 모달, 관리자 회원 승인 패널, 인증 API 요청 및 로그인 상태를 관리합니다. |
| `styles.css` | 랜딩 페이지, 인증 모달, 버튼 및 반응형 화면의 스타일을 정의합니다. |

### 프로젝트 루트 및 정적 파일

| 파일/폴더 | 역할 |
| --- | --- |
| `index.html` | 브라우저가 처음 읽는 HTML 진입점입니다. React 앱을 렌더링할 `#root`와 문서 제목·메타 정보를 포함합니다. |
| `package.json` | 프런트엔드 의존성과 `dev`, `build`, `preview` 명령을 정의합니다. |
| `package-lock.json` | npm 의존성 버전을 고정해 설치 결과를 재현할 수 있도록 합니다. |
| `vite.config.js` | Vite와 React 플러그인을 설정하고 개발 서버를 `0.0.0.0:5173`에서 실행하도록 지정합니다. |
| `.env.example` | 프런트엔드에서 사용할 환경 변수 예시입니다. |
| `public/` | 이미지와 동영상 등 브라우저에서 직접 제공되는 정적 자산을 보관합니다. |

## 사전 준비

- Node.js 18 이상 및 npm
- Python 3.10 이상
- PostgreSQL

## 로컬 실행 (Windows PowerShell)

### 1. PostgreSQL 데이터베이스 준비

PostgreSQL 서비스를 실행하고 `vlagraph` 데이터베이스를 만듭니다. 예를 들어 `postgres` 사용자로 생성하려면:

```powershell
createdb -U postgres vlagraph
```

프로젝트 루트에서 스키마를 적용합니다.

```powershell
psql -U postgres -d vlagraph -f backend\schema.sql
```

`createdb` 명령을 사용할 수 없다면 `psql`에 접속해 `CREATE DATABASE vlagraph;`를 실행한 다음 스키마 적용 명령을 실행하세요.

#### pgAdmin 4로 처음 설정하기

1. pgAdmin 4를 열고 **Object > Register > Server...**를 선택합니다.
2. **General** 탭의 **Name**에 `Local PostgreSQL` 등 알아볼 이름을 입력합니다.
3. **Connection** 탭에 PostgreSQL 설치 시 지정한 접속 정보를 입력합니다. 로컬 기본 설정 예시는 다음과 같습니다.
   - **Host name/address:** `localhost`
   - **Port:** `5432`
   - **Maintenance database:** `postgres`
   - **Username:** PostgreSQL 설치 시 만든 사용자 (예: `postgres`)
   - **Password:** 해당 사용자의 비밀번호
4. **Save**를 눌러 서버에 연결합니다. 연결에 실패하면 PostgreSQL 서비스가 실행 중인지, 호스트·포트·사용자·비밀번호가 맞는지 확인하세요.
5. 왼쪽 Browser에서 등록한 서버를 펼친 뒤 **Databases**를 마우스 오른쪽 버튼으로 클릭하고 **Create > Database...**를 선택합니다.
6. **Database**에 `vlagraph`를 입력하고 **Owner**에는 애플리케이션에서 사용할 PostgreSQL 사용자를 선택한 뒤 **Save**를 클릭합니다.
7. 새로 만든 `vlagraph` 데이터베이스를 선택한 뒤 **Tools > Query Tool**을 엽니다.
8. Query Tool에서 **Open File**을 눌러 프로젝트의 `backend\schema.sql` 파일을 열고 **Execute/Refresh** (F5)를 실행합니다.

실행 후 Browser에서 `vlagraph > Schemas > public > Tables`를 새로고침해 `users` 테이블이 생성되었는지 확인합니다. 이 테이블은 회원가입 시 저장될 계정 정보를 위한 것입니다.
기존 DB를 사용 중이라면 업데이트된 `backend\schema.sql`을 다시 실행해 비밀번호 재설정 토큰 테이블도 생성하세요. 기존 회원 데이터는 유지됩니다.

마지막으로 프로젝트 루트의 `.env`에서 `DATABASE_URL`이 위에서 만든 DB와 일치하는지 확인합니다. 예를 들어 사용자명이 `postgres`이고 PostgreSQL이 로컬 5432 포트에서 실행 중이면 다음과 같습니다.

```dotenv
DATABASE_URL=postgresql://postgres:<데이터베이스_비밀번호>@localhost:5432/vlagraph
```

`<데이터베이스_비밀번호>`를 해당 PostgreSQL 사용자의 실제 비밀번호로 바꾸세요. 비밀번호에 `@`, `:`, `/`, `#` 같은 URL 예약 문자가 포함되면 URL 인코딩이 필요합니다.

### 2. 환경 변수 설정

프로젝트 루트의 `.env` 파일을 만들거나 기존 파일을 수정합니다. 프런트엔드 설정과 백엔드 설정을 같은 파일에 둡니다.

```dotenv
VITE_API_BASE_URL=http://localhost:8001
DATABASE_URL=postgresql://postgres:<데이터베이스_비밀번호>@localhost:5432/vlagraph
SECRET_KEY=<32자_이상의_임의_문자열>
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=60
COOKIE_NAME=access_token
COOKIE_SECURE=false
FRONTEND_URL=http://localhost:5174
ADMIN_EMAILS=admin@example.com
ADMIN_INITIAL_PASSWORD=ChangeMe_123456
SMTP_HOST=smtp.example.com
SMTP_PORT=587
SMTP_USERNAME=<SMTP_사용자명>
SMTP_PASSWORD=<SMTP_비밀번호>
SMTP_FROM_EMAIL=noreply@example.com
```

`DATABASE_URL`을 실제 PostgreSQL 접속 정보에 맞게 수정하세요. `SECRET_KEY`에는 최소 32자의 임의 문자열을 사용합니다. 비밀번호 찾기를 사용하려면 SMTP 제공자의 호스트, 포트, 계정, 비밀번호, 발신 주소를 입력하고, `FRONTEND_URL`을 재설정 페이지가 열릴 웹 주소로 설정해야 합니다. 위 SMTP 주소는 예시이므로 실제 메일 제공자의 값으로 교체하세요. 백엔드는 실행 시 현재 디렉터리의 `.env`를 읽으므로, 아래 명령은 프로젝트 루트에서 실행해야 합니다. 환경 변수 전체 목록과 기본값은 [`backend/config.py`](backend/config.py)를 참고하세요.

### 3. 프런트엔드 의존성 설치

프로젝트 루트에서 실행합니다.

```powershell
npm ci
```

### 4. 백엔드 의존성 설치

프로젝트 루트에서 Python 가상 환경을 만들고 패키지를 설치합니다.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r backend\requirements.txt
```

### 5. 개발 서버 실행

두 개의 터미널을 열고, 두 명령 모두 프로젝트 루트에서 실행합니다.

**터미널 1 — 프런트엔드**

```powershell
npm run dev
```

**터미널 2 — 백엔드**

```powershell
.\.venv\Scripts\python.exe -m uvicorn backend.main:app --reload --port 8000
```

브라우저에서 다음 주소를 엽니다.

- 웹 페이지: <http://localhost:5174>
- 백엔드 API 문서: <http://localhost:8001/docs>

랜딩 페이지 확인에는 프런트엔드만 실행해도 됩니다. 회원가입과 로그인 기능을 사용하려면 프런트엔드와 백엔드, PostgreSQL이 모두 실행 중이어야 합니다.

## 주요 API

| 메서드 | 경로 | 설명 |
| --- | --- | --- |
| `POST` | `/api/auth/register` | 계정 생성 |
| `POST` | `/api/auth/login` | 로그인 |
| `POST` | `/api/auth/logout` | 로그아웃 |
| `GET` | `/api/auth/me` | 현재 로그인 사용자 조회 |
| `POST` | `/api/auth/password` | 현재 비밀번호 확인 후 비밀번호 변경 |
| `POST` | `/api/auth/password-reset/request` | 이메일로 만료 시간이 있는 비밀번호 재설정 링크 발송 |
| `POST` | `/api/auth/password-reset/confirm` | 재설정 링크의 토큰을 확인하고 새 비밀번호 설정 |

## 빌드

프런트엔드 프로덕션 빌드:

```powershell
npm run build
```

빌드 결과 미리보기:

```powershell
npm run preview
```
