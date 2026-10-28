import React, { useCallback, useEffect, useRef, useState } from 'react'
import { createRoot } from 'react-dom/client'
import './styles.css'

const ASSET_MAP = {
  automesLogo: '/images/automes-innovation-logo.png',
  topHero: '/images/digital-twin-hero.png',
  nvidiaLogo: '/images/nvidia-logo.png',
  unityLogo: '/images/unity-logo.png',
  ros2Logo: '/images/ros2-logo.png',
  referenceHome: '/images/reference-michel-home.png',
  demoVideo: '/videos/robot-workflow-demo.mp4',
}

const PARTNER_LOGOS = [
  { name: 'AutoMES Telstar', src: ASSET_MAP.automesLogo },
  { name: 'NVIDIA', src: ASSET_MAP.nvidiaLogo },
  { name: 'Unity', src: ASSET_MAP.unityLogo },
  { name: 'ROS 2', src: ASSET_MAP.ros2Logo }
]

const CAPABILITY_ITEMS = [
  { number: '1', title: 'Sensor Integration', text: 'Camera, LiDAR, RealSense가 한 번에 장면을 읽고 로봇 환경을 이해합니다.', image: '/images/features/01_sensor_integration.png' },
  { number: '2', title: 'VLA 모델 추론', text: '사용자 명령을 받아 로봇이 상황을 이해하고 행동을 계획합니다.', image: '/images/features/02_vla_model_inference.png' },
  { number: '3', title: 'Unity / Isaac Sim / ROS 제어', text: '시뮬레이션으로 로봇 행동을 검증하고 실제 매니퓰레이터에 연결합니다.', image: '/images/features/03_unity_isaacsim_ros.png' },
  { number: '4', title: '영상에서 모션 추출 → 휴머노이드 학습 데이터', text: '영상 속 동작을 추적해 휴머노이드 학습용 데이터셋으로 변환합니다.', image: '/images/features/04_motion_extraction_dataset.png' },
  { number: '5', title: 'FANUC LS / OMY3M / G1 로봇 데이터 처리', text: '로봇별 제어 방식과 명령 포맷을 표준 명령으로 통합합니다.', image: '/images/features/05_robot_data_processing.png' },
  { number: '6', title: '공간 메모리 + RAG 기반 공장 판단', text: '공장 환경과 과거 경험을 함께 고려해 작업 위치를 판단합니다.', image: '/images/features/06_spatial_memory_rag.png' },
  { number: '7', title: 'STT/TTS 음성 기반 로봇 명령', text: '음성 커맨드로 로봇을 제어하고 즉시 피드백을 반환합니다.', image: '/images/features/07_stt_tts_voice_command.png' },
  { number: '8', title: '그래프 템플릿 반복 실험', text: '기존 워크플로를 복제해 실험과 데모를 빠르게 반복합니다.', image: '/images/features/08_graph_template_experiments.png' }
]

const API_BASE_URL = import.meta.env.VITE_API_BASE_URL || 'http://localhost:8001'
const GRAPH_APP_URL = import.meta.env.VITE_GRAPH_APP_URL || 'http://localhost:9173'

const Icon = ({ name, size = 18 }) => {
  const paths = {
    arrow: (<><path d="M5 12h14" /><path d="m13 6 6 6-6 6" /></>),
    play: (<path d="m8 5 11 7-11 7V5Z" fill="currentColor" stroke="none" />),
    sensor: (<><rect x="3" y="5" width="18" height="14" rx="3" /><path d="M8 9h8M8 15h5" /><circle cx="16.5" cy="9.5" r="1.5" /></>),
    brain: (<><path d="M9 3a3 3 0 0 0-3 3v1.2A2.8 2.8 0 0 0 5 10v4a3 3 0 0 0 3 3h1.2A2.8 2.8 0 0 0 12 18v1.5A2.5 2.5 0 0 1 9.5 22h-1A2.5 2.5 0 0 1 6 19.5V18" /><path d="M15 3a3 3 0 0 1 3 3v1.2A2.8 2.8 0 0 1 19 10v4a3 3 0 0 1-3 3h-1.2A2.8 2.8 0 0 1 12 18v1.5A2.5 2.5 0 0 0 14.5 22h1A2.5 2.5 0 0 0 18 19.5V18" /><path d="M9 9h6M9 15h6" /></>),
    safety: (<><path d="M12 3 19 6v5c0 4.5-2.7 7.7-7 10-4.3-2.3-7-5.5-7-10V6l7-3Z" /><path d="m8.8 12.2 2.2 2.2 4.2-4.8" /></>),
    robot: (<><rect x="4" y="6" width="16" height="12" rx="3" /><path d="M12 6V3M8 13h.01M16 13h.01M8 18h8" /><path d="M2 11h2M20 11h2" /></>),
    check: (<path d="M5 12.5 9.5 17 19 7.5" fill="none" />)
  }

  return (
    <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      {paths[name]}
    </svg>
  )
}

function Reveal({ children, className = '', delay = 0 }) {
  const elementRef = useRef(null)

  useEffect(() => {
    const element = elementRef.current
    if (!element) return undefined

    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          element.classList.add('is-visible')
          observer.unobserve(element)
        }
      },
      { threshold: 0.14, rootMargin: '0px 0px -48px' }
    )

    observer.observe(element)
    return () => observer.disconnect()
  }, [])

  return (
    <div
      ref={elementRef}
      className={`scroll-reveal ${className}`}
      style={{ '--reveal-delay': `${delay}ms` }}
    >
      {children}
    </div>
  )
}

function AuthModal({ mode, onClose, onModeChange, onAuthenticated }) {
  const [form, setForm] = useState({ name: '', email: '', password: '', confirmPassword: '' })
  const [error, setError] = useState('')
  const [success, setSuccess] = useState('')
  const [isSubmitting, setIsSubmitting] = useState(false)
  const isSignup = mode === 'signup'
  const isForgotPassword = mode === 'forgot'
  const isPasswordReset = mode === 'reset'
  const resetToken = new URLSearchParams(window.location.search).get('reset_token')

  const switchMode = (nextMode) => {
    setError('')
    setSuccess('')
    onModeChange(nextMode)
  }

  const handleSubmit = async (event) => {
    event.preventDefault()
    setError('')
    setSuccess('')

    if (isPasswordReset && form.password !== form.confirmPassword) {
      setError('새 비밀번호가 서로 일치하지 않습니다.')
      return
    }

    setIsSubmitting(true)

    try {
      const endpoint = isSignup
        ? 'register'
        : isForgotPassword
          ? 'password-reset/request'
          : isPasswordReset
            ? 'password-reset/confirm'
            : 'login'
      const body = isForgotPassword
        ? { email: form.email }
        : isPasswordReset
          ? { token: resetToken, new_password: form.password }
          : form
      const response = await fetch(`${API_BASE_URL}/api/auth/${endpoint}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify(body)
      })
      const payload = await response.json().catch(() => ({}))
      if (!response.ok) {
        throw new Error(payload.detail || '요청을 처리하지 못했습니다.')
      }
      if (isForgotPassword) {
        setSuccess('해당 이메일로 계정이 존재하면 비밀번호 재설정 링크를 보냈습니다.')
      } else if (isPasswordReset) {
        const resetUrl = new URL(window.location.href)
        resetUrl.searchParams.delete('reset_token')
        window.history.replaceState({}, '', resetUrl)
        setSuccess('비밀번호를 변경했습니다. 새 비밀번호로 로그인해 주세요.')
      } else if (isSignup) {
        setSuccess('가입 신청이 접수되었습니다. 관리자 승인 후 로그인할 수 있습니다.')
        setForm({ name: '', email: '', password: '', confirmPassword: '' })
      } else {
        onAuthenticated(payload.user || payload)
        onClose()
      }
    } catch (requestError) {
      setError(requestError.message || '서버에 연결할 수 없습니다.')
    } finally {
      setIsSubmitting(false)
    }
  }

  return (
    <div className="auth-backdrop" role="presentation" onMouseDown={onClose}>
      <section className="auth-modal" role="dialog" aria-modal="true" aria-labelledby="auth-title" onMouseDown={(event) => event.stopPropagation()}>
        <button className="auth-close" type="button" onClick={onClose} aria-label="인증 창 닫기">×</button>
        <div className="auth-kicker">VLAGRAPH ACCOUNT</div>
        <h2 id="auth-title">
          {isSignup ? '회원가입' : isForgotPassword ? '비밀번호 찾기' : isPasswordReset ? '새 비밀번호 설정' : '로그인'}
        </h2>
        <p>
          {isSignup
            ? '로봇 AI 워크플로우를 설계할 계정을 만들어보세요.'
            : isForgotPassword
              ? '가입한 이메일 주소로 비밀번호 재설정 링크를 보내드립니다.'
              : isPasswordReset
                ? '새 비밀번호를 입력해 계정 비밀번호를 재설정하세요.'
                : 'VLAgraph 계정으로 계속 진행하세요.'}
        </p>
        {success && isPasswordReset ? (
          <div className="auth-success" role="status">{success}</div>
        ) : isPasswordReset && !resetToken ? (
          <div className="auth-error" role="alert">재설정 링크가 없거나 올바르지 않습니다. 비밀번호 찾기를 다시 요청해 주세요.</div>
        ) : (
        <form autoComplete="off" onSubmit={handleSubmit}>
          {isSignup && (
            <label>
              이름
              <input
                type="text"
                value={form.name}
                onChange={(event) => setForm({ ...form, name: event.target.value })}
                autoComplete="name"
                maxLength={80}
              />
            </label>
          )}
          {(isSignup || isForgotPassword || !isPasswordReset) && (
            <label>
              이메일
              <input
                type="email"
                value={form.email}
                onChange={(event) => setForm({ ...form, email: event.target.value })}
                autoComplete="email"
                required
              />
            </label>
          )}
          {!isForgotPassword && (
            <label>
              {isPasswordReset ? '새 비밀번호' : '비밀번호'}
              <input
                type="password"
                value={form.password}
                onChange={(event) => setForm({ ...form, password: event.target.value })}
                autoComplete={isSignup || isPasswordReset ? 'off' : 'current-password'}
                minLength={8}
                maxLength={128}
                required
              />
            </label>
          )}
          {isPasswordReset && (
            <label>
              새 비밀번호 확인
              <input
                type="password"
                value={form.confirmPassword}
                onChange={(event) => setForm({ ...form, confirmPassword: event.target.value })}
                autoComplete="off"
                minLength={8}
                maxLength={128}
                required
              />
            </label>
          )}
          {error && <div className="auth-error" role="alert">{error}</div>}
          {success && <div className="auth-success" role="status">{success}</div>}
          <button className="auth-submit" type="submit" disabled={isSubmitting}>
            {isSubmitting
              ? '처리 중...'
              : isSignup
                ? '계정 만들기'
                : isForgotPassword
                  ? '재설정 링크 보내기'
                  : isPasswordReset
                    ? '비밀번호 재설정'
                    : '로그인'}
          </button>
        </form>
        )}
        {isPasswordReset && success ? (
          <button className="auth-switch" type="button" onClick={() => switchMode('login')}>
            로그인 화면으로 이동
          </button>
        ) : isForgotPassword || isPasswordReset ? (
          <button className="auth-switch" type="button" onClick={() => switchMode('login')}>
            로그인으로 돌아가기
          </button>
        ) : (
          <>
            {!isSignup && (
              <button className="auth-switch" type="button" onClick={() => switchMode('forgot')}>
                비밀번호를 잊으셨나요?
              </button>
            )}
            <button className="auth-switch" type="button" onClick={() => switchMode(isSignup ? 'login' : 'signup')}>
              {isSignup ? '이미 계정이 있나요? 로그인' : '계정이 없나요? 회원가입'}
            </button>
          </>
        )}
      </section>
    </div>
  )
}

function UserProfileModal({ user, onClose }) {
  const [passwords, setPasswords] = useState({ current: '', next: '', confirm: '' })
  const [error, setError] = useState('')
  const [success, setSuccess] = useState('')
  const [isSubmitting, setIsSubmitting] = useState(false)

  const handlePasswordChange = async (event) => {
    event.preventDefault()
    setError('')
    setSuccess('')

    if (passwords.next !== passwords.confirm) {
      setError('새 비밀번호가 서로 일치하지 않습니다.')
      return
    }
    if (passwords.current === passwords.next) {
      setError('새 비밀번호는 현재 비밀번호와 달라야 합니다.')
      return
    }

    setIsSubmitting(true)
    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/password`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({
          current_password: passwords.current,
          new_password: passwords.next
        })
      })
      if (!response.ok) {
        const payload = await response.json().catch(() => ({}))
        throw new Error(payload.detail || '비밀번호를 변경하지 못했습니다.')
      }
      setPasswords({ current: '', next: '', confirm: '' })
      setSuccess('비밀번호가 변경되었습니다.')
    } catch (requestError) {
      setError(requestError.message || '서버에 연결할 수 없습니다.')
    } finally {
      setIsSubmitting(false)
    }
  }

  return (
    <div className="auth-backdrop" role="presentation" onMouseDown={onClose}>
      <section
        className="auth-modal profile-modal"
        role="dialog"
        aria-modal="true"
        aria-labelledby="profile-title"
        onMouseDown={(event) => event.stopPropagation()}
      >
        <button className="auth-close" type="button" onClick={onClose} aria-label="회원 정보 창 닫기">×</button>
        <div className="auth-kicker">VLAGRAPH ACCOUNT</div>
        <h2 id="profile-title">회원 정보</h2>
        <dl className="profile-details">
          <div>
            <dt>이름</dt>
            <dd>{user.name || '이름 미등록'}</dd>
          </div>
          <div>
            <dt>이메일</dt>
            <dd>{user.email}</dd>
          </div>
        </dl>
        <form className="profile-password-form" autoComplete="off" onSubmit={handlePasswordChange}>
          <h3>비밀번호 변경</h3>
          <label>
            현재 비밀번호
            <input
              type="password"
              value={passwords.current}
              onChange={(event) => setPasswords({ ...passwords, current: event.target.value })}
              autoComplete="current-password"
              minLength={8}
              maxLength={128}
              required
            />
          </label>
          <label>
            새 비밀번호
            <input
              type="password"
              value={passwords.next}
              onChange={(event) => setPasswords({ ...passwords, next: event.target.value })}
              autoComplete="off"
              minLength={8}
              maxLength={128}
              required
            />
          </label>
          <label>
            새 비밀번호 확인
            <input
              type="password"
              value={passwords.confirm}
              onChange={(event) => setPasswords({ ...passwords, confirm: event.target.value })}
              autoComplete="off"
              minLength={8}
              maxLength={128}
              required
            />
          </label>
          {error && <div className="auth-error" role="alert">{error}</div>}
          {success && <div className="auth-success" role="status">{success}</div>}
          <button className="auth-submit" type="submit" disabled={isSubmitting}>
            {isSubmitting ? '변경 중...' : '비밀번호 변경'}
          </button>
        </form>
        <button className="profile-close-button" type="button" onClick={onClose}>닫기</button>
      </section>
    </div>
  )
}

function AdminApprovalPanel() {
  const [users, setUsers] = useState([])
  const [isLoading, setIsLoading] = useState(true)
  const [busyUserId, setBusyUserId] = useState(null)
  const [error, setError] = useState('')
  const [notice, setNotice] = useState('')

  const loadUsers = useCallback(async () => {
    setIsLoading(true)
    setError('')
    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/admin/users`, {
        credentials: 'include'
      })
      const payload = await response.json().catch(() => ({}))
      if (!response.ok) {
        throw new Error(payload.detail || '회원 목록을 불러오지 못했습니다.')
      }
      setUsers(Array.isArray(payload) ? payload : [])
    } catch (requestError) {
      setError(requestError.message || '서버에 연결할 수 없습니다.')
    } finally {
      setIsLoading(false)
    }
  }, [])

  useEffect(() => {
    loadUsers()
  }, [loadUsers])

  const updateUserStatus = async (user, action) => {
    setBusyUserId(user.id)
    setError('')
    setNotice('')
    try {
      const response = await fetch(
        `${API_BASE_URL}/api/auth/admin/users/${encodeURIComponent(user.id)}/${action}`,
        { method: 'POST', credentials: 'include' }
      )
      const payload = await response.json().catch(() => ({}))
      if (!response.ok) {
        throw new Error(payload.detail || '회원 상태를 변경하지 못했습니다.')
      }
      setUsers((currentUsers) => currentUsers.map((item) => (
        item.id === payload.id ? payload : item
      )))
      setNotice(`${user.email} 계정을 ${action === 'approve' ? '승인' : '거절'}했습니다.`)
    } catch (requestError) {
      setError(requestError.message || '서버에 연결할 수 없습니다.')
    } finally {
      setBusyUserId(null)
    }
  }

  const pendingCount = users.filter((item) => item.status === 'pending').length

  return (
    <section className="admin-panel" aria-labelledby="admin-panel-title">
      <div className="admin-panel-header">
        <div>
          <div className="section-tag">ACCOUNT ADMINISTRATION</div>
          <h2 id="admin-panel-title">회원 가입 승인</h2>
          <p>가입 신청을 확인하고 서비스 이용을 승인하거나 거절합니다.</p>
        </div>
        <button className="admin-refresh-button" type="button" onClick={loadUsers} disabled={isLoading}>
          {isLoading ? '불러오는 중...' : '목록 새로고침'}
        </button>
      </div>

      <div className="admin-summary" aria-live="polite">
        <span>전체 회원 <strong>{users.length}</strong></span>
        <span>승인 대기 <strong>{pendingCount}</strong></span>
      </div>

      {error && <div className="auth-error admin-feedback" role="alert">{error}</div>}
      {notice && <div className="auth-success admin-feedback" role="status">{notice}</div>}

      {isLoading ? (
        <div className="admin-empty">회원 목록을 불러오는 중입니다.</div>
      ) : users.length === 0 && !error ? (
        <div className="admin-empty">등록된 회원이 없습니다.</div>
      ) : users.length > 0 && (
        <div className="admin-user-list">
          {users.map((account) => (
            <article className="admin-user-row" key={account.id}>
              <div className="admin-user-info">
                <strong>{account.name || '이름 미등록'}</strong>
                <span>{account.email}</span>
                <small>
                  가입일 {new Date(account.created_at).toLocaleDateString('ko-KR')}
                  {account.is_admin ? ' · 관리자' : ''}
                </small>
              </div>
              <div className="admin-user-controls">
                <span className={`admin-status admin-status-${account.status}`}>
                  {account.status === 'pending'
                    ? '승인 대기'
                    : account.status === 'approved'
                      ? '승인됨'
                      : '거절됨'}
                </span>
                {account.status !== 'approved' && (
                  <button
                    className="admin-approve-button"
                    type="button"
                    onClick={() => updateUserStatus(account, 'approve')}
                    disabled={busyUserId === account.id}
                  >
                    {busyUserId === account.id ? '처리 중...' : '승인'}
                  </button>
                )}
                {account.status === 'pending' && (
                  <button
                    className="admin-reject-button"
                    type="button"
                    onClick={() => updateUserStatus(account, 'reject')}
                    disabled={busyUserId === account.id}
                  >
                    {busyUserId === account.id ? '처리 중...' : '거절'}
                  </button>
                )}
              </div>
            </article>
          ))}
        </div>
      )}
    </section>
  )
}

function App() {
  const [authMode, setAuthMode] = useState(() => (
    new URLSearchParams(window.location.search).has('reset_token') ? 'reset' : null
  ))
  const [user, setUser] = useState(null)
  const [authLoaded, setAuthLoaded] = useState(false)
  const [showProfile, setShowProfile] = useState(false)
  const [simulationLaunching, setSimulationLaunching] = useState(false)
  const [simulationMessage, setSimulationMessage] = useState('')

  useEffect(() => {
    fetch(`${API_BASE_URL}/api/auth/me`, { credentials: 'include' })
      .then((response) => (response.ok ? response.json() : null))
      .then((payload) => {
        if (payload) setUser(payload.user || payload)
      })
      .catch(() => {})
      .finally(() => setAuthLoaded(true))
  }, [])

  const handleLogout = async () => {
    await fetch(`${API_BASE_URL}/api/auth/logout`, {
      method: 'POST',
      credentials: 'include'
    }).catch(() => {})
    setUser(null)
  }

  const handleLaunchSimulation = async () => {
    setSimulationLaunching(true)
    setSimulationMessage('')
    try {
      const response = await fetch(`${API_BASE_URL}/api/simulation/launch`, {
        method: 'POST',
        credentials: 'include',
      })
      const payload = await response.json().catch(() => ({}))
      if (!response.ok) {
        throw new Error(payload.detail || '시뮬레이션 폴더를 열지 못했습니다.')
      }
      setSimulationMessage(payload.message || '시뮬레이션 폴더를 열었습니다.')
    } catch (error) {
      setSimulationMessage(error.message || '시뮬레이션 폴더 실행 서비스에 연결할 수 없습니다.')
    } finally {
      setSimulationLaunching(false)
    }
  }

  return (
    <div className="app-shell">
      <div className="bg-watermark">VLA GRAHP</div>

      <header className="topbar">
        <div className="brand-wrap">
          <a href="http://automes.co.kr/" aria-label="AutoMES Innovation 홈페이지로 이동">
            <img src={ASSET_MAP.automesLogo} alt="AutoMES Innovation" className="brand-logo" />
          </a>
        </div>

        <div className="topbar-actions">
          {authLoaded && (
            user ? (
              <>
                <button className="user-greeting" type="button" onClick={() => setShowProfile(true)}>
                  {user.name || user.email}
                </button>
                <button className="ghost-btn" onClick={handleLogout}>로그아웃</button>
              </>
            ) : (
              <>
                <button className="ghost-btn login-btn" onClick={() => setAuthMode('login')}>로그인</button>
                <button className="primary-btn" onClick={() => setAuthMode('signup')}>회원가입</button>
              </>
            )
          )}
        </div>
      </header>

      {authLoaded && !user && (
        <>
          <section className="landing-hero" aria-label="AutoMES Telstar 제조공정 검증 플랫폼">
            <div className="landing-hero-backdrop" aria-hidden="true" />
          </section>

          <main className="page">
            <Reveal className="hero-reveal">
              <section className="hero-block">
          <div className="hero-copy">
            <h1>
              <span className="highlight">AI</span>와 로봇을 연결하는
              <br />
              지능형 운영 체계
            </h1>
            <p>
              다품종 생산, 로봇 프로그래밍, 안전 제어, 시뮬레이션 실행을 하나의 그래프로 통합하고
              실제 작업 환경에 바로 적용할 수 있는 운영 플랫폼입니다.
            </p>
          </div>
              </section>
            </Reveal>
          </main>
        </>
      )}

      {authLoaded && user && (
        <main className="page dashboard-page">
          {user.is_admin && <AdminApprovalPanel />}
          <Reveal>
            <section className="demo-entry">
              <div className="hero-side">
                <div className="mini-pill">AutoMES Telstar</div>
                <div className="demo-launch-grid">
                  <a
                    className="demo-launch-card"
                    href={GRAPH_APP_URL}
                    aria-label="VLAgraph 시연 화면 열기"
                  >
                    <span className="demo-launch-kicker">LIVE DEMO</span>
                    <span className="demo-launch-title">로봇 AI 워크플로우<br />시연 화면</span>
                    <span className="demo-launch-action">
                      시연 화면 열기
                      <Icon name="arrow" size={17} />
                    </span>
                  </a>
                  <button
                    className="simulation-launch-card"
                    type="button"
                    onClick={handleLaunchSimulation}
                    disabled={simulationLaunching}
                    aria-label="INDUSPHERE 실행"
                  >
                    <span className="simulation-launch-copy">
                      <span className="demo-launch-kicker">INDUSPHERE</span>
                      <strong>{simulationLaunching ? 'INDUSPHERE 실행 중...' : 'INDUSPHERE 실행'}</strong>
                    </span>
                    <span className="demo-launch-action">
                      <Icon name="play" size={17} />
                    </span>
                  </button>
                </div>
                {simulationMessage && <div className="simulation-launch-message">{simulationMessage}</div>}
              </div>
            </section>
          </Reveal>

        <Reveal>
          <section className="capability-gallery">
          <div className="feature-header-row">
            <div className="section-tag">Operational Layers</div>
            <div className="meta-links">
              <span>센서</span>
              <span>AI</span>
              <span>시뮬</span>
              <span>실행</span>
            </div>
          </div>

            <div className="capability-grid">
              {CAPABILITY_ITEMS.map((item, index) => (
                <Reveal className="capability-card-reveal" delay={index * 140} key={item.title}>
                  <article className="capability-card">
                <div className="capability-image-wrap">
                  <span className="capability-number">{item.number}</span>
                  <span className="capability-image-label">{item.title}</span>
                  <img src={item.image} alt={item.title} className="capability-image" />
                </div>
                <div className="capability-copy">
                  <h3>{item.title}</h3>
                  <p>{item.text}</p>
                </div>
                  </article>
                </Reveal>
              ))}
            </div>
          </section>
        </Reveal>

        <Reveal>
          <section className="feature-section">
          <div className="feature-header-row">
            <div className="section-tag">WHY VLAGRAPH</div>
            <div className="meta-links">
              <span>센서</span>
              <span>AI</span>
              <span>안전</span>
              <span>실행</span>
            </div>
          </div>

            <div className="feature-grid">
              {[
              { icon: 'sensor', title: 'Sensor-to-Action Graph', text: '카메라, LiDAR, 실시간 센서 입력을 그래프 노드로 연결해 운영 흐름을 시각화합니다.' },
              { icon: 'brain', title: 'VLA Model Orchestration', text: 'OpenVLA / NVIDIA GR00T를 포함한 모델 추론을 정교하게 연결해 실제 행동으로 변환합니다.' },
              { icon: 'safety', title: 'Safety-Constrained Execution', text: '작업 공간 제한, 안전 정책, 장애 대응을 설계 단계에서 반영해 실행 전 위험을 줄입니다.' }
              ].map((feature, index) => (
                <Reveal className="feature-card-reveal" delay={index * 100} key={feature.title}>
                  <article className="feature-card">
                <div className="feature-icon"><Icon name={feature.icon} size={18} /></div>
                <h3>{feature.title}</h3>
                <p>{feature.text}</p>
                  </article>
                </Reveal>
              ))}
            </div>
          </section>
        </Reveal>

        <Reveal>
          <section className="logo-row" aria-label="기술 파트너 로고">
          {PARTNER_LOGOS.map((logo) => (
            <img key={logo.name} src={logo.src} alt={logo.name} className="partner-logo" />
          ))}
          </section>
        </Reveal>
        </main>
      )}
      {authMode && (
        <AuthModal
          mode={authMode}
          onClose={() => setAuthMode(null)}
          onModeChange={setAuthMode}
          onAuthenticated={setUser}
        />
      )}
      {showProfile && user && (
        <UserProfileModal user={user} onClose={() => setShowProfile(false)} />
      )}
    </div>
  )
}

createRoot(document.getElementById('root')).render(<App />)
