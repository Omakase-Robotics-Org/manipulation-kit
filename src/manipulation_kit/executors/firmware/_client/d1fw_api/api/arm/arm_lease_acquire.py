from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_lease_acquire_response_200 import ArmLeaseAcquireResponse200
from ...models.arm_lease_request import ArmLeaseRequest
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    *,
    body: ArmLeaseRequest,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/arm/lease",
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ArmLeaseAcquireResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ArmLeaseAcquireResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ArmLeaseAcquireResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ArmLeaseRequest,
) -> Response[ArmLeaseAcquireResponse200 | ErrorEnvelope]:
    """Take, renew or preempt the operator lease on both arms

     Arbitration between the consumers that can all reach this daemon — a teleop session, a policy, and
    the robot's own ambient motion. While a lease is held, arm motion from anyone else is refused with
    409 naming the holder; reads, `estop`, `clear_errors`, `recover` and the soft kill are never
    arbitrated.

    Granted when the lease is free, when the current one has expired, or when you already hold it — that
    last case is the heartbeat, so renew by repeating this call well inside `ttl_s`. `ttl_s` defaults to
    30 seconds and may not exceed 600; expiry is evaluated when the lease is next read, so a consumer
    that dies frees the arms without anything having to notice it died.

    **Priority.** `class` is `ambient` (rank 10), `policy` (50) or `operator` (90), and **an omitted
    `class` means `ambient`** — the lowest privilege there is, so a consumer written before classes
    existed cannot outrank one written after. A request of a strictly HIGHER class than the holder
    preempts it: the answer carries `preempted_from` and a new `epoch`. An equal class is first-come
    (409), and a lower one is 409 too. `epoch` increments on every fresh grant and never on a heartbeat,
    so a consumer that reads its own name back can tell "I still hold these" from "I lost them and have
    them again" — and therefore whether what it believed about the arms is stale.

    **A preemption stops the arms first.** The displaced holder's trajectory is cancelled (`cancelled`,
    not `failed`) and both arms are taken exclusively before the grant is recorded, so the new holder
    never inherits a moving robot. The arm mode is deliberately NOT changed: a position or torque hold
    is where the arms are left. If the displaced holder has a command still in flight — a waited
    `move_joints` is the one that lasts — the hand-over gives up after 250 ms and the preemption is
    refused with 409 `hand-over timed out; the current holder has a command in flight`; the lease does
    not move. Stop the robot (`/v1/soft_kill`, `estop` — never leased) and ask again.

    There is no push notification: a displaced holder finds out by reading `GET /v1/arm/lease`
    (different `holder`, different `epoch`) or by having its next gated POST refused.

    The holder and the class are self-declared, not credentials: this daemon authenticates nobody, and
    the lease coordinates cooperating consumers rather than defending against a hostile one. Arbitration
    is opt-in — with no lease held every verb behaves exactly as it did before.

    Args:
        body (ArmLeaseRequest): `POST /v1/arm/lease`: take, renew, or preempt the arm lease.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmLeaseAcquireResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    body: ArmLeaseRequest,
) -> ArmLeaseAcquireResponse200 | ErrorEnvelope | None:
    """Take, renew or preempt the operator lease on both arms

     Arbitration between the consumers that can all reach this daemon — a teleop session, a policy, and
    the robot's own ambient motion. While a lease is held, arm motion from anyone else is refused with
    409 naming the holder; reads, `estop`, `clear_errors`, `recover` and the soft kill are never
    arbitrated.

    Granted when the lease is free, when the current one has expired, or when you already hold it — that
    last case is the heartbeat, so renew by repeating this call well inside `ttl_s`. `ttl_s` defaults to
    30 seconds and may not exceed 600; expiry is evaluated when the lease is next read, so a consumer
    that dies frees the arms without anything having to notice it died.

    **Priority.** `class` is `ambient` (rank 10), `policy` (50) or `operator` (90), and **an omitted
    `class` means `ambient`** — the lowest privilege there is, so a consumer written before classes
    existed cannot outrank one written after. A request of a strictly HIGHER class than the holder
    preempts it: the answer carries `preempted_from` and a new `epoch`. An equal class is first-come
    (409), and a lower one is 409 too. `epoch` increments on every fresh grant and never on a heartbeat,
    so a consumer that reads its own name back can tell "I still hold these" from "I lost them and have
    them again" — and therefore whether what it believed about the arms is stale.

    **A preemption stops the arms first.** The displaced holder's trajectory is cancelled (`cancelled`,
    not `failed`) and both arms are taken exclusively before the grant is recorded, so the new holder
    never inherits a moving robot. The arm mode is deliberately NOT changed: a position or torque hold
    is where the arms are left. If the displaced holder has a command still in flight — a waited
    `move_joints` is the one that lasts — the hand-over gives up after 250 ms and the preemption is
    refused with 409 `hand-over timed out; the current holder has a command in flight`; the lease does
    not move. Stop the robot (`/v1/soft_kill`, `estop` — never leased) and ask again.

    There is no push notification: a displaced holder finds out by reading `GET /v1/arm/lease`
    (different `holder`, different `epoch`) or by having its next gated POST refused.

    The holder and the class are self-declared, not credentials: this daemon authenticates nobody, and
    the lease coordinates cooperating consumers rather than defending against a hostile one. Arbitration
    is opt-in — with no lease held every verb behaves exactly as it did before.

    Args:
        body (ArmLeaseRequest): `POST /v1/arm/lease`: take, renew, or preempt the arm lease.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmLeaseAcquireResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ArmLeaseRequest,
) -> Response[ArmLeaseAcquireResponse200 | ErrorEnvelope]:
    """Take, renew or preempt the operator lease on both arms

     Arbitration between the consumers that can all reach this daemon — a teleop session, a policy, and
    the robot's own ambient motion. While a lease is held, arm motion from anyone else is refused with
    409 naming the holder; reads, `estop`, `clear_errors`, `recover` and the soft kill are never
    arbitrated.

    Granted when the lease is free, when the current one has expired, or when you already hold it — that
    last case is the heartbeat, so renew by repeating this call well inside `ttl_s`. `ttl_s` defaults to
    30 seconds and may not exceed 600; expiry is evaluated when the lease is next read, so a consumer
    that dies frees the arms without anything having to notice it died.

    **Priority.** `class` is `ambient` (rank 10), `policy` (50) or `operator` (90), and **an omitted
    `class` means `ambient`** — the lowest privilege there is, so a consumer written before classes
    existed cannot outrank one written after. A request of a strictly HIGHER class than the holder
    preempts it: the answer carries `preempted_from` and a new `epoch`. An equal class is first-come
    (409), and a lower one is 409 too. `epoch` increments on every fresh grant and never on a heartbeat,
    so a consumer that reads its own name back can tell "I still hold these" from "I lost them and have
    them again" — and therefore whether what it believed about the arms is stale.

    **A preemption stops the arms first.** The displaced holder's trajectory is cancelled (`cancelled`,
    not `failed`) and both arms are taken exclusively before the grant is recorded, so the new holder
    never inherits a moving robot. The arm mode is deliberately NOT changed: a position or torque hold
    is where the arms are left. If the displaced holder has a command still in flight — a waited
    `move_joints` is the one that lasts — the hand-over gives up after 250 ms and the preemption is
    refused with 409 `hand-over timed out; the current holder has a command in flight`; the lease does
    not move. Stop the robot (`/v1/soft_kill`, `estop` — never leased) and ask again.

    There is no push notification: a displaced holder finds out by reading `GET /v1/arm/lease`
    (different `holder`, different `epoch`) or by having its next gated POST refused.

    The holder and the class are self-declared, not credentials: this daemon authenticates nobody, and
    the lease coordinates cooperating consumers rather than defending against a hostile one. Arbitration
    is opt-in — with no lease held every verb behaves exactly as it did before.

    Args:
        body (ArmLeaseRequest): `POST /v1/arm/lease`: take, renew, or preempt the arm lease.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmLeaseAcquireResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: ArmLeaseRequest,
) -> ArmLeaseAcquireResponse200 | ErrorEnvelope | None:
    """Take, renew or preempt the operator lease on both arms

     Arbitration between the consumers that can all reach this daemon — a teleop session, a policy, and
    the robot's own ambient motion. While a lease is held, arm motion from anyone else is refused with
    409 naming the holder; reads, `estop`, `clear_errors`, `recover` and the soft kill are never
    arbitrated.

    Granted when the lease is free, when the current one has expired, or when you already hold it — that
    last case is the heartbeat, so renew by repeating this call well inside `ttl_s`. `ttl_s` defaults to
    30 seconds and may not exceed 600; expiry is evaluated when the lease is next read, so a consumer
    that dies frees the arms without anything having to notice it died.

    **Priority.** `class` is `ambient` (rank 10), `policy` (50) or `operator` (90), and **an omitted
    `class` means `ambient`** — the lowest privilege there is, so a consumer written before classes
    existed cannot outrank one written after. A request of a strictly HIGHER class than the holder
    preempts it: the answer carries `preempted_from` and a new `epoch`. An equal class is first-come
    (409), and a lower one is 409 too. `epoch` increments on every fresh grant and never on a heartbeat,
    so a consumer that reads its own name back can tell "I still hold these" from "I lost them and have
    them again" — and therefore whether what it believed about the arms is stale.

    **A preemption stops the arms first.** The displaced holder's trajectory is cancelled (`cancelled`,
    not `failed`) and both arms are taken exclusively before the grant is recorded, so the new holder
    never inherits a moving robot. The arm mode is deliberately NOT changed: a position or torque hold
    is where the arms are left. If the displaced holder has a command still in flight — a waited
    `move_joints` is the one that lasts — the hand-over gives up after 250 ms and the preemption is
    refused with 409 `hand-over timed out; the current holder has a command in flight`; the lease does
    not move. Stop the robot (`/v1/soft_kill`, `estop` — never leased) and ask again.

    There is no push notification: a displaced holder finds out by reading `GET /v1/arm/lease`
    (different `holder`, different `epoch`) or by having its next gated POST refused.

    The holder and the class are self-declared, not credentials: this daemon authenticates nobody, and
    the lease coordinates cooperating consumers rather than defending against a hostile one. Arbitration
    is opt-in — with no lease held every verb behaves exactly as it did before.

    Args:
        body (ArmLeaseRequest): `POST /v1/arm/lease`: take, renew, or preempt the arm lease.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmLeaseAcquireResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
