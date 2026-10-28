import hashlib
import logging
import secrets
import smtplib
import ssl
import json
from datetime import datetime, timedelta, timezone
from email.message import EmailMessage
from typing import Annotated
from uuid import UUID
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request as UrlRequest, urlopen

import jwt
import psycopg
from fastapi import Depends, FastAPI, HTTPException, Request, Response, status
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, ConfigDict, EmailStr, Field, field_validator
from pwdlib import PasswordHash

from .config import Settings, get_settings
from .db import get_connection

password_hash = PasswordHash.recommended()
logger = logging.getLogger(__name__)
app = FastAPI(title="VLA Graph Authentication API")
app.add_middleware(
    CORSMiddleware,
    allow_origin_regex=r"^https?://(localhost|127\.0\.0\.1)(:\d+)?$",
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type"],
)


class AuthCredentials(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True)

    email: EmailStr
    password: str = Field(min_length=8, max_length=128)


class RegisterRequest(AuthCredentials):
    name: str | None = Field(default=None, max_length=120)

    @field_validator("name")
    @classmethod
    def normalize_name(cls, value: str | None) -> str | None:
        if value is not None and not value:
            return None
        return value


class PasswordChangeRequest(BaseModel):
    current_password: str = Field(min_length=8, max_length=128)
    new_password: str = Field(min_length=8, max_length=128)


class PasswordResetRequest(BaseModel):
    email: EmailStr


class PasswordResetConfirmRequest(BaseModel):
    token: str = Field(min_length=32, max_length=128)
    new_password: str = Field(min_length=8, max_length=128)


class UserResponse(BaseModel):
    id: UUID
    email: EmailStr
    name: str | None
    status: str = "approved"
    is_admin: bool = False
    created_at: datetime


Connection = Annotated[psycopg.Connection, Depends(get_connection)]


def create_access_token(user_id: UUID, settings: Settings) -> str:
    now = datetime.now(timezone.utc)
    payload = {
        "sub": str(user_id),
        "iat": now,
        "exp": now + timedelta(minutes=settings.access_token_expire_minutes),
    }
    return jwt.encode(payload, settings.secret_key, algorithm=settings.jwt_algorithm)


def send_password_reset_email(email: str, token: str, settings: Settings) -> None:
    if not settings.smtp_host or not settings.smtp_from_email:
        raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="Password reset email is not configured")
    if bool(settings.smtp_username) != bool(settings.smtp_password):
        raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="SMTP credentials are incomplete")

    reset_url = f"{settings.frontend_url.rstrip('/')}/?reset_token={quote(token)}"
    message = EmailMessage()
    message["Subject"] = "VLAgraph 비밀번호 재설정"
    message["From"] = settings.smtp_from_email
    message["To"] = email
    message.set_content(
        "비밀번호 재설정 요청을 받았습니다.\n"
        "아래 링크에서 새 비밀번호를 설정하세요. 링크는 30분 동안 유효하며 한 번만 사용할 수 있습니다.\n\n"
        f"{reset_url}\n\n"
        "요청하지 않았다면 이 메일을 무시하세요."
    )

    with smtplib.SMTP(settings.smtp_host, settings.smtp_port, timeout=10) as server:
        server.starttls(context=ssl.create_default_context())
        if settings.smtp_username and settings.smtp_password:
            server.login(settings.smtp_username, settings.smtp_password)
        refused_recipients = server.send_message(message)
        if refused_recipients:
            raise smtplib.SMTPRecipientsRefused(refused_recipients)


def set_auth_cookie(response: Response, token: str, settings: Settings) -> None:
    response.set_cookie(
        key=settings.cookie_name,
        value=token,
        max_age=settings.access_token_expire_minutes * 60,
        httponly=True,
        secure=settings.cookie_secure,
        samesite="lax",
        path="/",
    )


def current_user(
    request: Request,
    connection: Connection,
) -> dict:
    settings: Settings = request.app.state.settings
    token = request.cookies.get(settings.cookie_name)
    if not token:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Authentication required")

    try:
        payload = jwt.decode(token, settings.secret_key, algorithms=[settings.jwt_algorithm])
        user_id = UUID(str(payload["sub"]))
    except (jwt.InvalidTokenError, KeyError, ValueError, TypeError):
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid or expired authentication token")

    user = connection.execute(
        "SELECT id, email, name, status, is_admin, created_at FROM users WHERE id = %s",
        (user_id,),
    ).fetchone()
    if user is None:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="User not found")
    if user["status"] != "approved":
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail="This account is not approved yet")
    return user


def require_admin(
    user: Annotated[dict, Depends(current_user)],
    connection: Connection,
) -> dict:
    account = connection.execute(
        "SELECT is_admin FROM users WHERE id = %s",
        (user["id"],),
    ).fetchone()
    if account is None or not account["is_admin"]:
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail="Admin access required")
    return user


def ensure_admin_accounts(settings: Settings) -> None:
    admin_emails = [email.strip().lower() for email in settings.admin_emails.split(",") if email.strip()]
    if not admin_emails or not settings.admin_initial_password:
        return

    with psycopg.connect(settings.database_url) as connection:
        for email in admin_emails:
            account = connection.execute(
                "SELECT id FROM users WHERE email = %s",
                (email,),
            ).fetchone()
            if account is None:
                connection.execute(
                    """
                    INSERT INTO users (email, name, password_hash, status, is_admin)
                    VALUES (%s, %s, %s, 'approved', true)
                    """,
                    (email, email.split("@")[0], password_hash.hash(settings.admin_initial_password)),
                )
            else:
                connection.execute(
                    "UPDATE users SET is_admin = true, status = 'approved' WHERE id = %s",
                    (account["id"],),
                )
        connection.commit()


@app.on_event("startup")
def configure_app() -> None:
    app.state.settings = get_settings()
    ensure_admin_accounts(app.state.settings)


@app.post("/api/auth/register", response_model=UserResponse, status_code=status.HTTP_201_CREATED)
def register(
    data: RegisterRequest,
    request: Request,
    connection: Connection,
) -> dict:
    email = str(data.email).lower()
    try:
        user = connection.execute(
            """
            INSERT INTO users (email, name, password_hash, status)
            VALUES (%s, %s, %s, 'pending')
            RETURNING id, email, name, status, is_admin, created_at
            """,
            (email, data.name, password_hash.hash(data.password)),
        ).fetchone()
        connection.commit()
    except psycopg.errors.UniqueViolation:
        connection.rollback()
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="An account with this email already exists")

    return user


@app.post("/api/auth/login", response_model=UserResponse)
def login(
    data: AuthCredentials,
    response: Response,
    request: Request,
    connection: Connection,
) -> dict:
    email = str(data.email).lower()
    user = connection.execute(
        "SELECT id, email, name, status, is_admin, created_at, password_hash FROM users WHERE email = %s",
        (email,),
    ).fetchone()
    if user is None or not password_hash.verify(data.password, user["password_hash"]):
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid email or password")
    if user["status"] == "pending":
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail="Your account is pending approval")
    if user["status"] == "rejected":
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail="Your account registration was rejected")

    set_auth_cookie(response, create_access_token(user["id"], request.app.state.settings), request.app.state.settings)
    return {key: user[key] for key in ("id", "email", "name", "status", "is_admin", "created_at")}


@app.post("/api/auth/logout", status_code=status.HTTP_204_NO_CONTENT)
def logout(response: Response, request: Request) -> None:
    response.delete_cookie(key=request.app.state.settings.cookie_name, path="/")


@app.get("/api/auth/me", response_model=UserResponse)
def me(user: Annotated[dict, Depends(current_user)]) -> dict:
    return user


@app.post("/api/simulation/launch")
def launch_omniverse_simulation(
    request: Request,
    user: Annotated[dict, Depends(current_user)],
) -> dict:
    del user
    settings: Settings = request.app.state.settings
    launcher_url = settings.omniverse_launcher_url
    launcher_token = settings.omniverse_launch_token

    if not launcher_url:
        raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="Omniverse launcher is not configured")
    if not launcher_token:
        raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="Omniverse launcher token is not configured")

    payload = json.dumps({}).encode("utf-8")
    launcher_request = UrlRequest(
        launcher_url,
        data=payload,
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {launcher_token}"},
        method="POST",
    )

    try:
        with urlopen(launcher_request, timeout=10) as response:
            result = json.loads(response.read().decode("utf-8"))
    except HTTPError as error:
        body = error.read().decode("utf-8", errors="replace")
        raise HTTPException(status_code=status.HTTP_502_BAD_GATEWAY, detail=f"Omniverse launcher error: {body[:300]}") from error
    except (URLError, TimeoutError, OSError) as error:
        raise HTTPException(status_code=status.HTTP_502_BAD_GATEWAY, detail="Omniverse launcher is not reachable") from error

    return result


@app.get("/api/auth/admin/users", response_model=list[UserResponse])
def list_users(
    admin: Annotated[dict, Depends(require_admin)],
    connection: Connection,
) -> list[dict]:
    del admin
    rows = connection.execute(
        "SELECT id, email, name, status, is_admin, created_at FROM users ORDER BY created_at DESC"
    ).fetchall()
    return rows


@app.post("/api/auth/admin/users/{user_id}/approve", response_model=UserResponse)
def approve_user(
    user_id: UUID,
    admin: Annotated[dict, Depends(require_admin)],
    connection: Connection,
) -> dict:
    del admin
    user = connection.execute(
        """
        UPDATE users
        SET status = 'approved'
        WHERE id = %s
        RETURNING id, email, name, status, is_admin, created_at
        """,
        (user_id,),
    ).fetchone()
    if user is None:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
    connection.commit()
    return user


@app.post("/api/auth/admin/users/{user_id}/reject", response_model=UserResponse)
def reject_user(
    user_id: UUID,
    admin: Annotated[dict, Depends(require_admin)],
    connection: Connection,
) -> dict:
    del admin
    user = connection.execute(
        """
        UPDATE users
        SET status = 'rejected'
        WHERE id = %s
        RETURNING id, email, name, status, is_admin, created_at
        """,
        (user_id,),
    ).fetchone()
    if user is None:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
    connection.commit()
    return user


@app.post("/api/auth/password", status_code=status.HTTP_204_NO_CONTENT)
def change_password(
    data: PasswordChangeRequest,
    user: Annotated[dict, Depends(current_user)],
    connection: Connection,
) -> None:
    account = connection.execute(
        "SELECT password_hash FROM users WHERE id = %s",
        (user["id"],),
    ).fetchone()
    if account is None or not password_hash.verify(data.current_password, account["password_hash"]):
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Current password is incorrect")
    if data.current_password == data.new_password:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="New password must be different")

    connection.execute(
        "UPDATE users SET password_hash = %s WHERE id = %s",
        (password_hash.hash(data.new_password), user["id"]),
    )
    connection.commit()


@app.post("/api/auth/password-reset/request", status_code=status.HTTP_202_ACCEPTED)
def request_password_reset(
    data: PasswordResetRequest,
    request: Request,
    connection: Connection,
) -> dict[str, str]:
    settings: Settings = request.app.state.settings
    if not settings.smtp_host or not settings.smtp_from_email:
        raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="Password reset email is not configured")
    if bool(settings.smtp_username) != bool(settings.smtp_password):
        raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="SMTP credentials are incomplete")

    email = str(data.email).lower()
    account = connection.execute("SELECT id FROM users WHERE email = %s", (email,)).fetchone()
    if account is not None:
        token = secrets.token_urlsafe(32)
        token_hash = hashlib.sha256(token.encode("utf-8")).hexdigest()
        connection.execute("DELETE FROM password_reset_tokens WHERE user_id = %s", (account["id"],))
        connection.execute(
            """
            INSERT INTO password_reset_tokens (token_hash, user_id, expires_at)
            VALUES (%s, %s, %s)
            """,
            (token_hash, account["id"], datetime.now(timezone.utc) + timedelta(minutes=30)),
        )
        connection.commit()

        mail_error: tuple[Exception, str] | None = None
        try:
            send_password_reset_email(email, token, settings)
        except smtplib.SMTPAuthenticationError as error:
            logger.exception("Gmail SMTP authentication failed while sending a password reset email")
            mail_error = (error, "Gmail rejected SMTP credentials; verify SMTP_USERNAME and SMTP_PASSWORD")
        except smtplib.SMTPRecipientsRefused as error:
            logger.exception("SMTP server rejected the password reset email recipient")
            mail_error = (error, "SMTP server rejected the recipient address")
        except (smtplib.SMTPException, OSError) as error:
            logger.exception("Password reset email delivery failed")
            mail_error = (error, "Unable to send the password reset email")

        if mail_error is not None:
            connection.execute("DELETE FROM password_reset_tokens WHERE token_hash = %s", (token_hash,))
            connection.commit()
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail=mail_error[1],
            ) from mail_error[0]

    return {"detail": "If an account exists for this email, a password reset link has been sent"}


@app.post("/api/auth/password-reset/confirm", status_code=status.HTTP_204_NO_CONTENT)
def confirm_password_reset(
    data: PasswordResetConfirmRequest,
    connection: Connection,
) -> None:
    if data.token == data.new_password:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="New password must be different")

    token_hash = hashlib.sha256(data.token.encode("utf-8")).hexdigest()
    reset = connection.execute(
        """
        SELECT user_id
        FROM password_reset_tokens
        WHERE token_hash = %s AND expires_at > now()
        FOR UPDATE
        """,
        (token_hash,),
    ).fetchone()
    if reset is None:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Password reset link is invalid or expired")

    connection.execute(
        "UPDATE users SET password_hash = %s WHERE id = %s",
        (password_hash.hash(data.new_password), reset["user_id"]),
    )
    connection.execute("DELETE FROM password_reset_tokens WHERE user_id = %s", (reset["user_id"],))
    connection.commit()
